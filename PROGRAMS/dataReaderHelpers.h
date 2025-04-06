#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include "Matrix.h"

#ifndef DATAREADER
#define DATAREADER

using std::string;
using std::vector;
// the directory of all the data files relative to the PROGRAMS folder
const string DATA_DIR = "./data";

/**
 * @brief get the next string of a filestream
 *
 * @param fileStream
 * @return string
 */
string read_string(std::ifstream* fileStream) {
    string nextStr = "";
    *fileStream >> nextStr;
    return nextStr;
}

/**
 * @brief read an int from a filestream
 *
 * @param fileStream
 * @return int
 */
int read_int(std::ifstream* fileStream) {
    string nextStr = "";
    *fileStream >> nextStr;
    return std::stoi(nextStr);
}

/**
 * @brief read a double from a filestream
 *
 * @param fileStream
 * @return double
 */
double read_double(std::ifstream* fileStream) {
    string nextStr = "";
    *fileStream >> nextStr;
    return std::stof(nextStr);
}

/**
 * @brief Get all data file names
 *
 * @param string data_dir: the directory of all the data files relative to the
 * PROGRAMS folder
 * @return vector<string>: list of file names in data_dir
 */
vector<string> get_all_fileNames(string data_dir) {
    vector<string> filenames;
    for(auto const& file : std::filesystem::directory_iterator(data_dir)) {
        string filename = file.path().generic_string();
        filenames.push_back(filename);
    }
    return filenames;
}

/**
 * @brief Get the number of test cases
 *
 * @return int: number of tests. Each debug or unknown test will have an output
 * file.
 */
int get_number_of_test_cases() {
    vector<string> files = get_all_fileNames(DATA_DIR);
    int num = 0;
    for(size_t i = 0; i < files.size(); i++) {
        if(files[i].find("calbody") != -1) {
            num++;
        }
    }
    return num;
}

/**
 * @brief this is a data structure for a calbody file
 *
 */
struct Calbody_data {
    string NAME;
    int N_D;
    int N_A;
    int N_C;
    double** d_i_coordinates;
    double** a_i_coordinates;
    double** c_i_coordinates;
};

/**
 * @brief reads all calbody data into a vector of Calbody_data structs, in
 * alphabetical order
 *
 * @return vector<Calbody_data>: a vector of all calbody datas in the data_dir
 */
vector<Calbody_data> read_calbody_data() {
    vector<Calbody_data> cal_data_vector;
    vector<string> filenames = get_all_fileNames(DATA_DIR);
    // filter filenames for calbody data
    for(size_t i = 0; i < filenames.size(); i++) {
        if(filenames[i].find("calbody") == -1) {
            filenames.erase(filenames.begin() + i);
            i--;
        }
    }
    // for each calbody file, create a struct and add it to the return vector
    for(string filename : filenames) {
        Calbody_data cal_data;
        std::ifstream* CalbodyFile = new std::ifstream(filename);
        int fileNameStart = filename.find_last_of("/") + 1;
        int fileNameEnd = filename.find("calbody.txt");
        cal_data.NAME =
            filename.substr(fileNameStart, fileNameEnd - fileNameStart);
        string nextStr = "";
        cal_data.N_D = read_int(CalbodyFile);
        cal_data.N_A = read_int(CalbodyFile);
        cal_data.N_C = read_int(CalbodyFile);
        filename = read_string(CalbodyFile);
        // get d_i_coordinates
        cal_data.d_i_coordinates =
            (double**)malloc(sizeof(double*) * cal_data.N_D);
        for(int i = 0; i < cal_data.N_D; i++) {
            cal_data.d_i_coordinates[i] = (double*)malloc(sizeof(double) * 3);
        }
        for(int i = 0; i < cal_data.N_D; i++) {
            cal_data.d_i_coordinates[i][0] = read_double(CalbodyFile);
            cal_data.d_i_coordinates[i][1] = read_double(CalbodyFile);
            cal_data.d_i_coordinates[i][2] = read_double(CalbodyFile);
        }
        // get a_i_coordinates
        cal_data.a_i_coordinates =
            (double**)malloc(sizeof(double*) * cal_data.N_A);
        for(int i = 0; i < cal_data.N_A; i++) {
            cal_data.a_i_coordinates[i] = (double*)malloc(sizeof(double) * 3);
        }
        for(int i = 0; i < cal_data.N_A; i++) {
            cal_data.a_i_coordinates[i][0] = read_double(CalbodyFile);
            cal_data.a_i_coordinates[i][1] = read_double(CalbodyFile);
            cal_data.a_i_coordinates[i][2] = read_double(CalbodyFile);
        }
        // get c_i_coordinates
        cal_data.c_i_coordinates =
            (double**)malloc(sizeof(double*) * cal_data.N_C);
        for(int i = 0; i < cal_data.N_C; i++) {
            cal_data.c_i_coordinates[i] = (double*)malloc(sizeof(double) * 3);
        }
        for(int i = 0; i < cal_data.N_C; i++) {
            cal_data.c_i_coordinates[i][0] = read_double(CalbodyFile);
            cal_data.c_i_coordinates[i][1] = read_double(CalbodyFile);
            cal_data.c_i_coordinates[i][2] = read_double(CalbodyFile);
        }
        cal_data_vector.push_back(cal_data);
        CalbodyFile->close();
        delete CalbodyFile;
    }

    return cal_data_vector;
}

/**
 * @brief this is a data structure for a calreadings file
 *
 */
struct Calreadings_data {
    string NAME;
    int N_D;
    int N_A;
    int N_C;
    int N_Frames;
    double**** frames;
};

/**
 * @brief reads all calreadings data into a vector of Calreadings_data structs,
 * in alphabetical order
 *
 * @return vector<Calreadings_data>: a vector of all calreadings datas in the
 * data_dir
 */
vector<Calreadings_data> read_calreadings_data() {
    vector<Calreadings_data> cal_data_vector;
    vector<string> filenames = get_all_fileNames(DATA_DIR);
    // filter filenames for calreadings data
    for(size_t i = 0; i < filenames.size(); i++) {
        if(filenames[i].find("calreadings") == -1) {
            filenames.erase(filenames.begin() + i);
            i--;
        }
    }
    // for each file, create a struct and add it to the return vector
    for(string filename : filenames) {
        Calreadings_data cal_data;
        std::ifstream* CalreadingsFile = new std::ifstream(filename);
        int fileNameStart = filename.find_last_of("/") + 1;
        int fileNameEnd = filename.find("calreadings.txt");
        cal_data.NAME =
            filename.substr(fileNameStart, fileNameEnd - fileNameStart);
        string nextStr = "";
        cal_data.N_D = read_int(CalreadingsFile);
        cal_data.N_A = read_int(CalreadingsFile);
        cal_data.N_C = read_int(CalreadingsFile);
        cal_data.N_Frames = read_int(CalreadingsFile);
        filename = read_string(CalreadingsFile);
        cal_data.frames =
            (double****)malloc(sizeof(double***) * cal_data.N_Frames);
        for(int i = 0; i < cal_data.N_Frames; i++) {
            cal_data.frames[i] = (double***)malloc(sizeof(double**) * 3);
            cal_data.frames[i][0] =
                (double**)malloc(sizeof(double*) * cal_data.N_D);
            for(int j = 0; j < cal_data.N_D; j++) {
                cal_data.frames[i][0][j] = (double*)malloc(sizeof(double) * 3);
            }
            for(int j = 0; j < cal_data.N_D; j++) {
                cal_data.frames[i][0][j][0] = read_double(CalreadingsFile);
                cal_data.frames[i][0][j][1] = read_double(CalreadingsFile);
                cal_data.frames[i][0][j][2] = read_double(CalreadingsFile);
            }
            cal_data.frames[i][1] =
                (double**)malloc(sizeof(double*) * cal_data.N_A);
            for(int j = 0; j < cal_data.N_A; j++) {
                cal_data.frames[i][1][j] = (double*)malloc(sizeof(double) * 3);
            }
            for(int j = 0; j < cal_data.N_A; j++) {
                cal_data.frames[i][1][j][0] = read_double(CalreadingsFile);
                cal_data.frames[i][1][j][1] = read_double(CalreadingsFile);
                cal_data.frames[i][1][j][2] = read_double(CalreadingsFile);
            }
            cal_data.frames[i][2] =
                (double**)malloc(sizeof(double*) * cal_data.N_C);
            for(int j = 0; j < cal_data.N_C; j++) {
                cal_data.frames[i][2][j] = (double*)malloc(sizeof(double) * 3);
            }
            for(int j = 0; j < cal_data.N_C; j++) {
                cal_data.frames[i][2][j][0] = read_double(CalreadingsFile);
                cal_data.frames[i][2][j][1] = read_double(CalreadingsFile);
                cal_data.frames[i][2][j][2] = read_double(CalreadingsFile);
            }
        }
        cal_data_vector.push_back(cal_data);
        CalreadingsFile->close();
        delete CalreadingsFile;
    }

    return cal_data_vector;
}

/**
 * @brief this is a data structure for a EMPIVOT file
 *
 */
struct EMPivot_data {
    string NAME;
    int N_G;
    int N_Frames;
    double*** frames;
};

/**
 * @brief reads all empivot data into a vector of EMPivot_data structs, in
 * alphabetical order
 *
 * @return vector<EMPivot>: a vector of all empivot datas in the data_dir
 */
vector<EMPivot_data> read_empivot_data() {
    vector<EMPivot_data> cal_data_vector;
    vector<string> filenames = get_all_fileNames(DATA_DIR);
    // filter filenames for empivot data
    for(size_t i = 0; i < filenames.size(); i++) {
        if(filenames[i].find("empivot") == -1) {
            filenames.erase(filenames.begin() + i);
            i--;
        }
    }
    // for each file, create a struct and add it to the return vector
    for(string filename : filenames) {
        EMPivot_data empivot_data;
        std::ifstream* EMPivotFile = new std::ifstream(filename);
        int fileNameStart = filename.find_last_of("/") + 1;
        int fileNameEnd = filename.find("empivot.txt");
        empivot_data.NAME =
            filename.substr(fileNameStart, fileNameEnd - fileNameStart);
        string nextStr = "";
        empivot_data.N_G = read_int(EMPivotFile);
        empivot_data.N_Frames = read_int(EMPivotFile);
        filename = read_string(EMPivotFile);
        empivot_data.frames =
            (double***)malloc(sizeof(double**) * empivot_data.N_Frames);
        for(int i = 0; i < empivot_data.N_Frames; i++) {
            empivot_data.frames[i] =
                (double**)malloc(sizeof(double*) * empivot_data.N_G);
            for(int j = 0; j < empivot_data.N_G; j++) {
                empivot_data.frames[i][j] = (double*)malloc(sizeof(double) * 3);
            }
            for(int j = 0; j < empivot_data.N_G; j++) {
                empivot_data.frames[i][j][0] = read_double(EMPivotFile);
                empivot_data.frames[i][j][1] = read_double(EMPivotFile);
                empivot_data.frames[i][j][2] = read_double(EMPivotFile);
            }
        }
        cal_data_vector.push_back(empivot_data);
        EMPivotFile->close();
        delete EMPivotFile;
    }

    return cal_data_vector;
}

/**
 * @brief this is a data structure for a optpivot file
 *
 */
struct OPTPivot_data {
    string NAME;
    int N_D;
    int N_H;
    int N_Frames;
    double**** frames;
};

/**
 * @brief reads all OPTPivot data into a vector of OPTPivot_data structs, in
 * alphabetical order
 *
 * @return vector<OPTPivot_data>: a vector of all calreadings datas in the
 * data_dir
 */
vector<OPTPivot_data> read_optpivot_data() {
    vector<OPTPivot_data> opt_data_vector;
    vector<string> filenames = get_all_fileNames(DATA_DIR);
    // filter filenames for optpivot data
    for(size_t i = 0; i < filenames.size(); i++) {
        if(filenames[i].find("optpivot") == -1) {
            filenames.erase(filenames.begin() + i);
            i--;
        }
    }
    // for each file, create a struct and add it to the return vector
    for(string filename : filenames) {
        OPTPivot_data opt_data;
        std::ifstream* OPTPivotFile = new std::ifstream(filename);
        int fileNameStart = filename.find_last_of("/") + 1;
        int fileNameEnd = filename.find("optpivot.txt");
        opt_data.NAME =
            filename.substr(fileNameStart, fileNameEnd - fileNameStart);
        string nextStr = "";
        opt_data.N_D = read_int(OPTPivotFile);
        opt_data.N_H = read_int(OPTPivotFile);
        opt_data.N_Frames = read_int(OPTPivotFile);
        filename = read_string(OPTPivotFile);
        opt_data.frames =
            (double****)malloc(sizeof(double***) * opt_data.N_Frames);
        for(int i = 0; i < opt_data.N_Frames; i++) {
            opt_data.frames[i] = (double***)malloc(sizeof(double**) * 2);
            opt_data.frames[i][0] =
                (double**)malloc(sizeof(double*) * opt_data.N_D);
            for(int j = 0; j < opt_data.N_D; j++) {
                opt_data.frames[i][0][j] = (double*)malloc(sizeof(double) * 3);
            }
            for(int j = 0; j < opt_data.N_D; j++) {
                opt_data.frames[i][0][j][0] = read_double(OPTPivotFile);
                opt_data.frames[i][0][j][1] = read_double(OPTPivotFile);
                opt_data.frames[i][0][j][2] = read_double(OPTPivotFile);
            }
            opt_data.frames[i][1] =
                (double**)malloc(sizeof(double*) * opt_data.N_H);
            for(int j = 0; j < opt_data.N_H; j++) {
                opt_data.frames[i][1][j] = (double*)malloc(sizeof(double) * 3);
            }
            for(int j = 0; j < opt_data.N_H; j++) {
                opt_data.frames[i][1][j][0] = read_double(OPTPivotFile);
                opt_data.frames[i][1][j][1] = read_double(OPTPivotFile);
                opt_data.frames[i][1][j][2] = read_double(OPTPivotFile);
            }
        }
        opt_data_vector.push_back(opt_data);
        OPTPivotFile->close();
        delete OPTPivotFile;
    }
    return opt_data_vector;
}

/**
 * @brief this is a data structure for a CT-fiducials file
 *
 */
struct CT_Fiducials_data {
    string NAME;
    int N_B;
    double** b_i_coordinates;
};

/**
 * @brief reads all CT-fiducial data into a vector of CT_Fiducials_data structs,
 * in alphabetical order
 *
 * @return vector<CT_Fiducials_data>: a vector of all ct-fiducials datas in the
 * data_dir
 */
vector<CT_Fiducials_data> read_ct_fiducials_data() {
    vector<CT_Fiducials_data> ct_data_vector;
    vector<string> filenames = get_all_fileNames(DATA_DIR);
    // filter filenames for ct fiducials data
    for(size_t i = 0; i < filenames.size(); i++) {
        if(filenames[i].find("ct-fiducials") == -1) {
            filenames.erase(filenames.begin() + i);
            i--;
        }
    }
    // for each ct-fiducials file, create a struct and add it to the return
    // vector
    for(string filename : filenames) {
        CT_Fiducials_data ct_data;
        std::ifstream* CtFidFile = new std::ifstream(filename);
        int fileNameStart = filename.find_last_of("/") + 1;
        int fileNameEnd = filename.find("ct-fiducials.txt");
        ct_data.NAME =
            filename.substr(fileNameStart, fileNameEnd - fileNameStart);
        string nextStr = "";
        ct_data.N_B = read_int(CtFidFile);
        filename = read_string(CtFidFile);
        // get b_i_coordinates
        ct_data.b_i_coordinates =
            (double**)malloc(sizeof(double*) * ct_data.N_B);
        for(int i = 0; i < ct_data.N_B; i++) {
            ct_data.b_i_coordinates[i] = (double*)malloc(sizeof(double) * 3);
        }
        for(int i = 0; i < ct_data.N_B; i++) {
            ct_data.b_i_coordinates[i][0] = read_double(CtFidFile);
            ct_data.b_i_coordinates[i][1] = read_double(CtFidFile);
            ct_data.b_i_coordinates[i][2] = read_double(CtFidFile);
        }
        ct_data_vector.push_back(ct_data);
        CtFidFile->close();
        delete CtFidFile;
    }
    return ct_data_vector;
}

/**
 * @brief this is a data structure for a em-fiducials file
 *
 */
struct EM_Fiducials_data {
    string NAME;
    int N_G;
    int N_B;
    double*** frames;
};

/**
 * @brief reads all EM-fiducials data into a vector of EM_Fiducials_data
 * structs, in alphabetical order
 *
 * @return vector<EM_Fiducials_data>: a vector of all em-fiducials datas in the
 * data_dir
 */
vector<EM_Fiducials_data> read_emfiducials_data() {
    vector<EM_Fiducials_data> em_data_vector;
    vector<string> filenames = get_all_fileNames(DATA_DIR);
    // filter filenames for emfiducials data
    for(size_t i = 0; i < filenames.size(); i++) {
        if(filenames[i].find("em-fiducialss") == -1) {
            filenames.erase(filenames.begin() + i);
            i--;
        }
    }
    // for each file, create a struct and add it to the return vector
    for(string filename : filenames) {
        EM_Fiducials_data em_data;
        std::ifstream* EMFidFile = new std::ifstream(filename);
        int fileNameStart = filename.find_last_of("/") + 1;
        int fileNameEnd = filename.find("em-fiducialss.txt");
        em_data.NAME =
            filename.substr(fileNameStart, fileNameEnd - fileNameStart);
        string nextStr = "";
        em_data.N_G = read_int(EMFidFile);
        em_data.N_B = read_int(EMFidFile);
        filename = read_string(EMFidFile);
        em_data.frames = (double***)malloc(sizeof(double**) * em_data.N_B);
        for(int i = 0; i < em_data.N_B; i++) {
            em_data.frames[i] = (double**)malloc(sizeof(double*) * em_data.N_G);
            for(int j = 0; j < em_data.N_G; j++) {
                em_data.frames[i][j] = (double*)malloc(sizeof(double) * 3);
            }
            for(int j = 0; j < em_data.N_G; j++) {
                em_data.frames[i][j][0] = read_double(EMFidFile);
                em_data.frames[i][j][1] = read_double(EMFidFile);
                em_data.frames[i][j][2] = read_double(EMFidFile);
            }
        }
        em_data_vector.push_back(em_data);
        EMFidFile->close();
        delete EMFidFile;
    }
    return em_data_vector;
}

/**
 * @brief this is a data structure for a EM-nav file
 *
 */
struct EM_Nav_data {
    string NAME;
    int N_G;
    int N_Frames;
    double*** frames;
};

/**
 * @brief reads all EM-nav data into a vector of EM_nav structs, in alphabetical
 * order
 *
 * @return vector<EM_Nav_data>: a vector of all em-nav datas in the data_dir
 */
vector<EM_Nav_data> read_emnav_data() {
    vector<EM_Nav_data> em_data_vector;
    vector<string> filenames = get_all_fileNames(DATA_DIR);
    // filter filenames for EM-nav data
    for(size_t i = 0; i < filenames.size(); i++) {
        if(filenames[i].find("EM-nav") == -1) {
            filenames.erase(filenames.begin() + i);
            i--;
        }
    }
    // for each file, create a struct and add it to the return vector
    for(string filename : filenames) {
        EM_Nav_data em_data;
        std::ifstream* EMNavFile = new std::ifstream(filename);
        int fileNameStart = filename.find_last_of("/") + 1;
        int fileNameEnd = filename.find("EM-nav.txt");
        em_data.NAME =
            filename.substr(fileNameStart, fileNameEnd - fileNameStart);
        string nextStr = "";
        em_data.N_G = read_int(EMNavFile);
        em_data.N_Frames = read_int(EMNavFile);
        filename = read_string(EMNavFile);
        em_data.frames = (double***)malloc(sizeof(double**) * em_data.N_Frames);
        for(int i = 0; i < em_data.N_Frames; i++) {
            em_data.frames[i] = (double**)malloc(sizeof(double*) * em_data.N_G);
            for(int j = 0; j < em_data.N_G; j++) {
                em_data.frames[i][j] = (double*)malloc(sizeof(double) * 3);
            }
            for(int j = 0; j < em_data.N_G; j++) {
                em_data.frames[i][j][0] = read_double(EMNavFile);
                em_data.frames[i][j][1] = read_double(EMNavFile);
                em_data.frames[i][j][2] = read_double(EMNavFile);
            }
        }
        em_data_vector.push_back(em_data);
        EMNavFile->close();
        delete EMNavFile;
    }
    return em_data_vector;
}

/**
 * @brief make sure all pointers are freed when done
 *
 * @param calbody_data
 * @param calreadings_data
 * @param empivot_data
 * @param opt_data
 * @param ctf_data
 * @param em_fid_data
 * @param em_nav_data
 */
void clear_readings(Calbody_data calbody_data,
                    Calreadings_data calreadings_data,
                    EMPivot_data empivot_data, OPTPivot_data opt_data,
                    CT_Fiducials_data ctf_data, EM_Fiducials_data em_fid_data,
                    EM_Nav_data em_nav_data) {
    // clear calbody_data
    for(int i = 0; i < calbody_data.N_D; i++) {
        free(calbody_data.d_i_coordinates[i]);
    }
    free(calbody_data.d_i_coordinates);
    for(int i = 0; i < calbody_data.N_C; i++) {
        free(calbody_data.c_i_coordinates[i]);
    }
    free(calbody_data.c_i_coordinates);
    for(int i = 0; i < calbody_data.N_A; i++) {
        free(calbody_data.a_i_coordinates[i]);
    }
    free(calbody_data.a_i_coordinates);

    // clear calreadings data
    for(int i = 0; i < calreadings_data.N_Frames; i++) {
        for(int j = 0; j < calreadings_data.N_D; j++) {
            free(calreadings_data.frames[i][0][j]);
        }
        for(int j = 0; j < calreadings_data.N_A; j++) {
            free(calreadings_data.frames[i][1][j]);
        }
        for(int j = 0; j < calreadings_data.N_C; j++) {
            free(calreadings_data.frames[i][2][j]);
        }
        free(calreadings_data.frames[i][0]);
        free(calreadings_data.frames[i][1]);
        free(calreadings_data.frames[i][2]);
        free(calreadings_data.frames[i]);
    }
    free(calreadings_data.frames);

    // clear EMPivot_data
    for(int i = 0; i < empivot_data.N_Frames; i++) {
        for(int j = 0; j < empivot_data.N_G; j++) {
            free(empivot_data.frames[i][j]);
        }
        free(empivot_data.frames[i]);
    }
    free(empivot_data.frames);

    // clear optpivot data
    for(int i = 0; i < opt_data.N_Frames; i++) {
        for(int j = 0; j < opt_data.N_D; j++) {
            free(opt_data.frames[i][0][j]);
        }
        for(int j = 0; j < opt_data.N_H; j++) {
            free(opt_data.frames[i][1][j]);
        }
        free(opt_data.frames[i][0]);
        free(opt_data.frames[i][1]);
        free(opt_data.frames[i]);
    }
    free(opt_data.frames);

    // clear ct fiducials data
    for(int i = 0; i < ctf_data.N_B; i++) {
        free(ctf_data.b_i_coordinates[i]);
    }
    free(ctf_data.b_i_coordinates);

    // clear em fiducials data
    for(int i = 0; i < em_fid_data.N_B; i++) {
        for(int j = 0; j < em_fid_data.N_G; j++) {
            free(em_fid_data.frames[i][j]);
        }
        free(em_fid_data.frames[i]);
    }
    free(em_fid_data.frames);

    // clear em-nav data
    for(int i = 0; i < em_nav_data.N_Frames; i++) {
        for(int j = 0; j < em_nav_data.N_G; j++) {
            free(em_nav_data.frames[i][j]);
        }
        free(em_nav_data.frames[i]);
    }
    free(em_nav_data.frames);
}

/**
 * @brief Create a output file for a debugging set
 *
 * @param v_i_list
 * @param file_name
 * @param N
 */
void create_output_file(vector<Matrix> v_i_list, string file_name, int N) {
    file_name += "OUTPUT2.txt";
    std::ofstream outfile("../OUTPUT/" + file_name);
    if(!outfile.is_open()) {
        throw std::logic_error("The file wasn't opened");
    }
    outfile << v_i_list.size() << ", " << file_name << "\n";
    for(int i = 0; i < v_i_list.size(); i++) {
        outfile << v_i_list[i].matrixArray[0] << ", "
                << v_i_list[i].matrixArray[1] << ", "
                << v_i_list[i].matrixArray[2] << std::endl;
    }
}

/**
 * @brief Out put the mean squared error for a debugging set
 *
 * @param filename
 */
void output_average_error(string filename) {
    filename += "OUTPUT2.txt";

    std::ifstream* debugfile = new std::ifstream("./data/" + filename);
    std::ifstream* outputfile = new std::ifstream("../OUTPUT/" + filename);
    int numCoords = read_int(debugfile);
    numCoords = read_int(outputfile);
    filename = read_string(debugfile);
    filename = read_string(outputfile);
    double sum = 0;
    for(int i = 0; i < 3 * numCoords; i++) {
        sum += pow(read_double(outputfile) - read_double(debugfile), 2);
    }
    std::cout << "Mean squared error for file " << filename << ": " << sum / 12
              << std::endl;
}
#endif