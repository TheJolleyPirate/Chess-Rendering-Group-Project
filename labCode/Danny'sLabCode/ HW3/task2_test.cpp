#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/core/base.hpp>
#include <opencv2/opencv.hpp>
#include <include/mesh.hpp>
#include <vector>


int main(int argc, const char **argv) {
         
/*
    This is an example test code, 
    you can use this code to test your implementation of task2 using your own test cases.
    We will mark your testing strategy based on your report
    Any modification to this code will not affect the grading.
    We will mark your task2 using the same coding structure as this file  with our hidden tests.
*/
    std::string file_path;
    
    Mesh mesh;
    // check the task 2
    std::vector<std::pair<std::string, std::string>> test_cases_1 = {
        {"../model/test1.obj", "N"},
        {"../model/test2.obj", "YN"},
        {"../model/test3.obj", "YYN"},
        {"../model/test4.obj", "YYYN"},
        {"../model/test5.obj", "YYYYN"},
        {"../model/test6.obj", "YYYYY"},
    };
    // check the task 2 extension 4.1
    std::vector<std::pair<std::string, std::string>> test_cases_2 = {
        {"../model/test7.obj", "NN"},
        {"../model/test8.obj", "NYN"},
        {"../model/test9.obj", "NYYN"},
        {"../model/test10.obj", "NYYYN"},
        {"../model/test11.obj", "NYYYY"}
    };

   
    int passed_count_1 = 0;
    std::string passed_files_1 = "";
    for (const auto& test_case : test_cases_1) {
        std::string file_path = test_case.first;
        std::string expected_result = test_case.second;
        std::string output = mesh.check_mesh(file_path);
        if (output.substr(0, expected_result.size()) == expected_result) {
           passed_count_1++;
           passed_files_1 += file_path + " \n";
        } 
    }
    std::cout << "Passed test cases for task 2: " << passed_count_1 << "/" << test_cases_1.size() << "\n";
    std::cout << "Passed files: \n" << passed_files_1 << "\n";

    int passed_count_2 = 0;
    std::string passed_files_2 = "";
    for (const auto& test_case : test_cases_2) {
        std::string file_path = test_case.first;
        std::string expected_result = test_case.second;
        std::string output = mesh.check_mesh(file_path);
        if (output.substr(0, expected_result.size()) == expected_result) {
           passed_count_1++;
           passed_files_1 += file_path + " \n";
        } 
    }
    std::cout << "Passed test cases for task 2 extension: " << passed_count_2 << "/" << test_cases_2.size() << "\n";
    std::cout << "Passed files: \n" << passed_files_2 << "\n";

    return 0;
}
