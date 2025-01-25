// Author: Calabrese Federico
// Purpose: 
// This program performs camera calibration to remove distortion introduced by the camera lens.
// Lens distortion is a common issue in images captured by cameras, especially wide-angle lenses.
// The calibration process calculates the intrinsic and extrinsic parameters of the camera,
// allowing the removal of radial and tangential distortions.
//
// References:
// - OpenCV Documentation: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
// - Additional Resources:
//   * https://learnopencv.com/camera-calibration-using-opencv/
//   * https://markhedleyjones.com/projects/calibration-checkerboard-collection

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>

#include <ros/ros.h>

using namespace cv;
using namespace std;

namespace fs = std::filesystem;

int main() {
    // Definiamo la checkerboard
    Size boardSize(8, 6);
    float squareSize = 0.025; // Metri (prima era 2.5)

    // Immagaziniamo i punti 3D e 2D per ogni immagine
    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;

    // Definiamo le coordinate globali della checkerboard in 3D
    vector<Point3f> obj;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            obj.push_back(Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    // Percorso della directory contenente le immagini della checkerboard catturate con la fotocamera
    string cameraName = "logitech_c270_hd_webcam";
    // string cameraName = "philips_spz_3000";
    string cameraDir = "/home/federico/Desktop/catkin_ws_prova/src/op2_visual/calibration/logitech_c270_hd_webcam/";
    // string cameraDir = "/home/federico/Desktop/catkin_ws/src/op2_visual/calibration/philips_spz_3000/";
    string inputDir = cameraDir + "*.jpg";
    string cornerOutputDir = cameraDir + "corner/";
    string undistortedOutputDir = cameraDir + "undistorted/";

    // Caricamento delle immagini dalla directory specificata dal percorso hard-coded
    vector<String> images;
    glob(inputDir, images);
    Mat frame, gray;
    
    for (auto imagePath : images) {
        frame = imread(imagePath);
        if (frame.empty()) {
            cout << "Image not found: " << imagePath << endl;
            continue;
        }

        // Convertiamo in scala di grigi
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        vector<Point2f> corners;

        // Rilevamento degli angoli della checkerboard
        bool found = findChessboardCorners(gray, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            // Raffina le posizioni dei punti
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            // Save points
            imagePoints.push_back(corners);
            objectPoints.push_back(obj);

            // Disegna i corners
            drawChessboardCorners(frame, boardSize, corners, found);

            // Estrae il nome originale del file e la sua estensione
            string originalFilename = fs::path(imagePath).stem().string();
            string extension = fs::path(imagePath).extension().string();

            // Crea il nome con cui verrà salvata l'immagine processata
            string outputPath = cornerOutputDir + originalFilename + "_with_corners" + extension;

            // Salva l'immagine processata
            imwrite(outputPath, frame);
            cout << "Processed image saved to: " << outputPath << endl;

            // imshow("Chessboard Detection", frame);
            // waitKey(100);
        }
    }
    // destroyAllWindows();

    // Calibrazione della Fotocamera
    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    double rms = calibrateCamera(objectPoints, imagePoints, gray.size(), cameraMatrix, distCoeffs, rvecs, tvecs);
    cout << "RMS error reported by calibrateCamera: " << rms << endl;
    cout << "Camera matrix: \n" << cameraMatrix << endl;
    cout << "Distortion coefficients: \n" << distCoeffs << endl;

    // Salva i risultati su di un file .yml
    string filePath = cameraDir + cameraName + "_params.yml";
    FileStorage fs(filePath, FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs.release();
    cout << "Calibration parameters saved to " << filePath << endl;

    // Salva le immagini prive di distorsione
    for (auto imagePath : images) {
        frame = imread(imagePath);
        if (frame.empty()) {
            cout << "Image not found: " << imagePath << endl;
            continue;
        }

        Mat undistorted;
        undistort(frame, undistorted, cameraMatrix, distCoeffs);

        // Estrae il nome originale del file e la sua estensione
        string originalFilename = fs::path(imagePath).stem().string();
        string extension = fs::path(imagePath).extension().string();

        // Crea il nome con cui verrà salvata l'immagine processata
        string outputPath = undistortedOutputDir + originalFilename + "_undistorted" + extension;

        // Salva l'immagine processata
        imwrite(outputPath, undistorted);
        cout << "Processed image saved to: " << outputPath << endl;

        // imshow("Undistorted Image", undistorted);
        // waitKey(100);
    }

    return 0;
}
