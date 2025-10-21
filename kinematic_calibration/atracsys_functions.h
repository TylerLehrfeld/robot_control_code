#ifndef ATRACSYS_FUNCTIONS
#define ATRACSYS_FUNCTIONS

#include "ftkInterface.h"
#include <thread>
#include <chrono>
#include "geometryHelper.hpp"
#include "helpers.hpp"
#include "kinematics.h"
#include <cmath>
#include <iostream>
#include <map>
#include <string>

float MAX_REGISTRATION_ERROR = .4;

enum atracsys_bitmask { BOTH, FOM1, FOM2 };

// static std::string M1_file = "../scripts/geometry100000.ini";
// static std::string M2_file = "../scripts/geometry999.ini";

template <typename T>
T atracsys_transform_to_templated_transform(ftkMarker *marker,
                                            Transform<T> &transform) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      transform.R.matrix[i][j] = T(marker->rotation[i][j]);
    }
  }
  transform.p.x = T(marker->translationMM[0]);
  transform.p.y = T(marker->translationMM[1]);
  transform.p.z = T(marker->translationMM[2]);
  return marker->registrationErrorMM;
}

template <typename T> class AtracsysTracker {
public:
  AtracsysTracker(const std::string &M1_file, const std::string &M2_file)
      : M1_file(M1_file), M2_file(M2_file) {
    ftkBuffer buffer;
    lib = ftkInitExt(nullptr, &buffer);
    if (lib == nullptr) {
      throw std::runtime_error(std::string("Failed to init Atracsys: ") +
                               buffer.data);
    }

    device = DeviceData(retrieveLastDevice(lib, true, false, true));
    sn = device.SerialNumber;

    if (ftkEnumerateOptions(lib, sn, optionEnumerator, &options) !=
            ftkError::FTK_OK ||
        options.empty()) {
      throw std::runtime_error(
          "Cannot retrieve any options from Atracsys device.");
    }
    // Build full path for registration
    std::string M1_full_path =
        "/home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/"
        "fusionTrack_SDK-v4.10.1-linux64/data/" +
        M1_file;
    if (loadRigidBody(lib, M1_file, M1) < 0) {
      throw std::runtime_error("Couldn't load M1 file: " + M1_file);
    }
    // Register M1 with device using full path
    if (ftkSetRigidBody(lib, sn, &M1) != ftkError::FTK_OK) {
      throw std::runtime_error("Couldn't register M1 with device: " +
                               M1_full_path);
    }

    // Load M2 using just filename
    if (loadRigidBody(lib, M2_file, M2) < 0) {
      throw std::runtime_error("Couldn't load M2 file: " + M2_file);
    }

    // Build full path for registration
    std::string M2_full_path =
        "/home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/"
        "fusionTrack_SDK-v4.10.1-linux64/data/" +
        M2_file;

    // Register M2 with device using full path
    if (ftkSetRigidBody(lib, sn, &M2) != ftkError::FTK_OK) {
      throw std::runtime_error("Couldn't register M2 with device: " +
                               M2_full_path);
    }
    //    std::string path_1 =
    //        ("/home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/"
    //         "fusionTrack_SDK-v4.10.1-linux64/data/" +
    //         M1_file);
    //
    //    std::string path_2 =
    //        ("/home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/"
    //         "fusionTrack_SDK-v4.10.1-linux64/data/" +
    //         M2_file);
    //    if (loadRigidBody(lib, M1_file, M1) != 1) {
    //      throw std::runtime_error("Couldn't load M1 file.");
    //    }
    //    if (loadRigidBody(lib, path_2, M2) != 1) {
    //      throw std::runtime_error("Couldn't load M2 file.");
    //    }
    //    // Register M1 with the device
    //    if (ftkRegisterRigidBody(lib, sn, path_1.size(), path_2.c_str()) !=
    //        ftkError::FTK_OK) {
    //      throw std::runtime_error("Couldn't register M1 with device.");
    //    }
    //    // Register M1 with the device
    //    if (ftkRegisterRigidBody(lib, sn, path_2.size(), path_2.c_str()) !=
    //        ftkError::FTK_OK) {
    //      throw std::runtime_error("Couldn't register M1 with device.");
    //    }
  }

  int getMeasurement(enum atracsys_bitmask mask, Measurement<T> &measurement) {
    ftkFrameQuery *frame = ftkCreateFrame();
    if (!frame) {
      std::cerr << "Cannot create frame instance" << std::endl;
      return -1;
    }

    if (ftkSetFrameOptions(false, false, 0u, 0u, 0u, 4u, frame) !=
        ftkError::FTK_OK) {
      ftkDeleteFrame(frame);
      std::cerr << "Could not initialize frame." << std::endl;
      return -1;
    }

    int tries = 0;
    while (tries++ < 50) {
      if (ftkGetLastFrame(lib, sn, frame, 1000) == ftkError::FTK_OK &&
          frame->markersCount > 0) {
        break;
      }
      std::cerr << "frame not found" << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    ftkMarker *markerOne = nullptr;
    ftkMarker *markerTwo = nullptr;

    for (int i = 0; i < frame->markersCount; ++i) {
      if ((mask == BOTH || mask == FOM1) &&
          frame->markers[i].geometryId == M1.geometryId) {
        markerOne = &frame->markers[i];
      }
      if ((mask == BOTH || mask == FOM2) &&
          frame->markers[i].geometryId == M2.geometryId) {
        markerTwo = &frame->markers[i];
      }
    }
    if (!markerOne) {
      std::cerr << "Warning: M1 marker not visible this frame\n";
      return -1;
    }
    if (!markerTwo) {
      std::cerr << "Warning: M2 marker not visible this frame\n";
      return -1;
    }

    if ((mask == FOM1 && !markerOne) || (mask == FOM2 && !markerTwo) ||
        (mask == BOTH && !markerOne) || (mask == BOTH && !markerTwo)) {
      ftkDeleteFrame(frame);
      return -1;
    }
    float err1 = 0.f, err2 = 0.f;
    if (mask == FOM1 || mask == BOTH && markerOne != nullptr) {
      err1 = atracsys_transform_to_templated_transform(markerOne,
                                                       measurement.F_OM1);
    }
    if (mask == FOM2 || mask == BOTH && markerTwo != nullptr) {
      err2 = atracsys_transform_to_templated_transform(markerTwo,
                                                       measurement.F_OM2);
    }

    ftkDeleteFrame(frame);

    if (err1 > MAX_REGISTRATION_ERROR || err2 > MAX_REGISTRATION_ERROR) {
      std::cerr << "error too high on marker reading" << std::endl;
      return -1;
    }

    return 0;
  }

private:
  ftkLibrary lib;
  DeviceData device;
  uint64 sn;
  ftkRigidBody M1, M2;
  std::map<std::string, uint32> options;
  std::string M1_file, M2_file;
};

// template <typename T>
// int get_atracsys_measurement(enum atracsys_bitmask mask,
//                              Measurement<T> &atracsys_measurement) {
//   ftkBuffer buffer;
//   ftkLibrary lib(ftkInitExt(nullptr, &buffer));
//   if (lib == nullptr) {
//     std::cerr << buffer.data << std::endl;
//     return -1;
//   }
//
//   DeviceData device(retrieveLastDevice(lib, true, false, true));
//   uint64 sn(device.SerialNumber);
//
//   std::map<std::string, uint32> options{};
//
//   ftkError err(ftkEnumerateOptions(lib, sn, optionEnumerator, &options));
//   if (options.empty()) {
//     std::cout << "Cannot retrieve any options." << std::endl;
//     return -1;
//   }
//   ftkRigidBody M1{};
//   ftkRigidBody M2{};
//   if (mask == BOTH || mask == FOM1) {
//     if (loadRigidBody(lib, M1_file, M1) != 1) {
//       std::cout << "Couldn't load M2 file." << std::endl;
//       return -1;
//     }
//   }
//   if (mask == BOTH || mask == FOM2) {
//
//     if (loadRigidBody(lib, M2_file, M2) != 1) {
//       std::cout << "Couldn't load M2 file." << std::endl;
//       return -1;
//     }
//   }
//
//   ftkFrameQuery *frame = ftkCreateFrame();
//   if (frame == nullptr) {
//     std::cout << "Cannot create frame instance" << std::endl;
//     return -1;
//   }
//
//   if (ftkError::FTK_OK !=
//       ftkSetFrameOptions(false, false, 0u, 0u, 0u, 4u, frame)) {
//     ftkDeleteFrame(frame);
//     std::cout << "Could not initialize frame." << std::endl;
//     return -1;
//   }
//
//   ftkGetLastFrame(lib, sn, frame, 200);
//
//   ftkMarker *markerOne = 0, *markerTwo = 0;
//   int i, k;
//   for (i = 0u; i < frame->markersCount; ++i) {
//     if ((mask == BOTH || mask == FOM1) &&
//         frame->markers[i].geometryId == M1.geometryId) {
//       markerOne = &(frame->markers[i]);
//     } else if ((mask == BOTH || mask == FOM1) &&
//                frame->markers[i].geometryId == M2.geometryId) {
//       markerTwo = &(frame->markers[i]);
//     }
//   }
//
//   if ((mask == FOM1 && markerOne == 0) || (mask == FOM2 && markerTwo == 0)) {
//     std::cout << "At least one marker is missing from data." << std::endl;
//     sleep(1000L);
//     err = ftkGetLastFrame(lib, sn, frame, 1000);
//     return -1;
//   }
//   Measurement<double> M;
//   float err1, err2 = 0;
//   if ((mask == FOM1 || mask == BOTH)) {
//     err1 = atracsys_transform_to_templated_transform(
//         markerOne, atracsys_measurement.F_OM1);
//   }
//   if ((mask == FOM2 || mask == BOTH)) {
//     err2 = atracsys_transform_to_templated_transform(
//         markerTwo, atracsys_measurement.F_OM2);
//   }
//   if (err1 > MAX_REGISTRATION_ERROR) {
//     std::cout << "Marker one registration error too high. " << err1 << " > "
//               << MAX_REGISTRATION_ERROR << std::endl;
//     return -1;
//   }
//   if (err2 > MAX_REGISTRATION_ERROR) {
//     std::cout << "Marker two registration error too high. " << err2 << " > "
//               << MAX_REGISTRATION_ERROR << std::endl;
//     return -1;
//   }
//   ftkDeleteFrame(frame);
//   return 0;
// }

#endif // !define ATRACSYS_FUNCTIONS
