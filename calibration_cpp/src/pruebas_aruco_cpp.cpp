#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

int main(int argc, char **argv) {

    cv::Mat inputImage = cv::imread("/home/tfg/dectmani_ws/src/camera_py/camera_py/im_color.jpg");
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    // Supongamos que inputImage es tu imagen de entrada
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(inputImage, dictionary, corners, ids);

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 509.0920104980469, 0, 299.10101318359375, 0, 509.0920104980469, 244.7949981689453, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
    float markerLength = 0.18;

    cv::Vec3d object_point(0.21422511376652564, 0.2234554803447643, 1.379);

    // Set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

    // Si al menos un marcador se detecta
    if (!ids.empty()) {

        std::cout << "si!" << std::endl;
        cv::aruco::drawDetectedMarkers(inputImage, corners, ids);
        cv::imshow("out", inputImage);
        cv::waitKey(0);
        
        int nMarkers = corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        // Calcular pose para cada marcador
        for (int i = 0; i < nMarkers; i++) {

            cv::solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            
        }

        // Dibujar ejes para cada marcador
        for (unsigned int i = 0; i < ids.size(); i++) {
            cv::aruco::drawAxis(inputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }

        cv::imshow("Image", inputImage);
        cv::waitKey(0);
        cv::destroyAllWindows();

    } else {
        std::cout << "no!" << std::endl;
    }

    return 0;
}