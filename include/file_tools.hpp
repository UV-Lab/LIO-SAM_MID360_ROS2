#pragma once

/////////////////////////////////// SC-Tools ///////////////////////////////////
#include "sc/Scancontext.h"
#include <fstream>
#include <iomanip>

enum class SCInputType { SINGLE_SCAN_FULL, SINGLE_SCAN_FEAT, MULTI_SCAN_FEAT };

inline void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ") {
    // delimiter: ", " or " " etc.
    int precision = 3;  // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");

    std::ofstream file(fileName);
    if (file.is_open()) {
        file << matrix.format(the_format);
        file.close();
    }
}

inline std::string padZeros(int val, int num_digits = 6) {
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

// zxl(已完成)
inline void loadSCD(std::string fileName, Eigen::MatrixXd& matrix, char delimiter = ' ') {
    // delimiter: ", " or " " etc.
    std::vector<double> matrixEntries;
    std::ifstream matrixDataFile(fileName);
    if (!matrixDataFile.is_open()) {
        std::cout << "读入SCD文件失败!!" << std::endl;
        return;
    }

    std::string matrixRowString;
    std::string matrixEntry;
    int matrixRowNumber = 0;
    while (getline(matrixDataFile, matrixRowString)) {
        std::stringstream matrixRowStringStream(matrixRowString);

        while (getline(matrixRowStringStream, matrixEntry, delimiter)) {
            matrixEntries.push_back(stod(matrixEntry));
        }
        matrixRowNumber++;
    }

    matrix = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber,
                                                                                                matrixEntries.size() / matrixRowNumber);
    matrixDataFile.close();
}

// zxl(已完成)
inline void loadPoses(std::string fileName, Eigen::MatrixXd& matrixPose, char delimiter = ' ') {
    // delimiter: ", " or " " etc.
    std::vector<double> matrixEntries;
    std::ifstream matrixDataFile(fileName);
    if (!matrixDataFile.is_open()) {
        std::cout << "读入SCD文件失败!!" << std::endl;
        return;
    }

    std::string matrixRowString;
    std::string matrixEntry;
    int matrixRowNumber = 0;
    while (getline(matrixDataFile, matrixRowString)) {
        std::stringstream matrixRowStringStream(matrixRowString);

        while (getline(matrixRowStringStream, matrixEntry, delimiter)) {
            matrixEntries.push_back(stod(matrixEntry));
        }
        matrixRowNumber++;
    }

    matrixPose = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber,
                                                                                                    matrixEntries.size() / matrixRowNumber);
    matrixDataFile.close();
}

/////////////////////////////////// File-Tools ///////////////////////////////////
#include <sys/stat.h>  // mkdir, stat

inline bool createDirectoryIfNotExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        // Directory does not exist, create it
        if (mkdir(path.c_str(), 0755) == 0) {
            return true;
        } else {
            std::cerr << "Error creating directory: " << path << std::endl;
            return false;
        }
    } else if (info.st_mode & S_IFDIR) {
        // Directory exists
        return true;
    } else {
        // Path exists but is not a directory
        std::cerr << "Path exists but is not a directory: " << path << std::endl;
        return false;
    }
}