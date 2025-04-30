#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/core/base.hpp>
#include <opencv2/opencv.hpp>
#include <include/mesh.hpp>

constexpr double MY_PI = 3.1415926;

int main(int argc, const char **argv) {
     
    // obtain model path and simlification ratio from argument and define output model path
    // for example, if you have you exec program built under ./task1 under ${root}/build, and models are in ${root}/model,
    // use sample command "./task2 ../model/bigguy2.obj 0.1" to test your algorithm.
    
    std::string file_path;
    if (argc >= 2) {
        file_path = std::string(argv[1]);
    } else{
      std::cout << "Please Input your model path as an argument";
      return 0;
    }
    Mesh mesh;
    std::string output = mesh.check_mesh(file_path);
    return 0;
}
