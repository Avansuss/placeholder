# Setup OpenCV Starter
## 1. Maak een projectmap
Kies een locatie en maak een map, noem het bijvoorbeeld “opencv”:
```
mkdir ~/opencv
cd ~/opencv
```
## 2. Maak een CMake-bestand
Maak een bestand CMakeLists.txt in de map opencv:
```
nano CMakeLists.txt
```
En zet dit er in: 
```
cmake_minimum_required(VERSION 3.10)
project(ArucoCamera)

set(CMAKE_CXX_STANDARD 23)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
```
Tip: CMake gebruikt dit bestand om je project te configureren en de juiste libraries te linken.

## 3. Maak een build-map
CMake gebruikt een aparte build-map. Maak die aan:
```
mkdir build
```
Tip: hiermee blijft je broncode netjes gescheiden van de build-bestanden.

## 4. Maak een testscript
Maak een nieuw C++ script, bijvoorbeeld “TestOpenCV.cpp”:
```
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    return 0;
}
```
Tip: dit script controleert eenvoudig of OpenCV correct geïnstalleerd is.

## 5. Voeg het script toe aan CMakeLists.txt
Open je CMakeLists.txt en voeg voor elk script een executable toe. Voor TestOpenCV.cpp:
```
add_executable(Test TestOpenCV.cpp)
target_link_libraries(Test ${OpenCV_LIBS})
```
Algemeen: 
```
add_executable({EIGEN_BENAMING} {NAAM_SCRIPT}.cpp)
target_link_libraries({EIGEN_BENAMING} ${OpenCV_LIBS})
```

Tip: kies een korte, herkenbare naam (Test in dit geval), zodat je later makkelijk kunt runnen.

## 6. Build het project
Ga naar de build-map:
```
cd build
```
Run CMake en make:
```
cmake ..
make
```

Tip: Als dit succesvol is, zijn er geen syntax-fouten. Elke keer dat je een nieuw script toevoegt of iets wijzigt, herhaal je deze stap.

## 7. Run je script
Run het script met de naam die je in CMakeLists.txt hebt opgegeven:
```
./Test
```

Je zou zoiets moeten zien als:
OpenCV version: x.x.x

Algemeen: 
```
./{EIGEN_BENAMING}
```

# OpenCV Project
Om live beeld te zien terwijl je via SSH verbinding maakt met de robot, moet je gebruikmaken van X11 Forwarding. X11 is het systeem achter grafische vensters op Linux. Met X11 Forwarding kun je grafische applicaties op een remote machine draaien, terwijl de vensters op je lokale computer worden weergegeven via SSH.

In dit geval wil je je script op de robot uitvoeren, maar het beeld lokaal op je laptop zien. Daarom is X11 Forwarding nodig. Zonder X11 Forwarding werkt een script dat een GUI opent niet via SSH, omdat er geen “display” beschikbaar is.

Om in te loggen met X11 Forwarding gebruik je bijvoorbeeld:
```
ssh -X rens@<IP-ADRES>
```

## CameraAruco.cpp: 
Dit script gebruikt de camera om ArUco-markers in real-time te detecteren en hun positie en rotatie te schatten. Elke marker heeft een eigen ID die wordt herkent en in de output is te zien. X11 Forwarding nodig!
```
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0, CAP_V4L2);

    if(!cap.isOpened()) {
        cerr << "Error: Kan camera niet openen" << endl;
        return -1;
    }

    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CAP_PROP_FRAME_HEIGHT, 240);

    Mat frame;
    namedWindow("Camera", WINDOW_NORMAL);
    resizeWindow("Camera", 160, 120);

    // ArUco dictionary en detector parameters
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();

    // Camera calibration, voorbeeldwaarden
    Mat cameraMatrix = (Mat_<double>(3,3) << 300, 0, 160, 0, 300, 120, 0, 0, 1);
    Mat distCoeffs = Mat::zeros(5,1,CV_64F);

    float markerLength = 0.05f; // Marker grootte in meter (of een referentie eenheid)

    while(true) {
        cap >> frame;
        if(frame.empty()) continue;

        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners, rejectedCandidates;

        // Detecteer markers
        aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        if(markerIds.size() > 0) {
            // Teken bounding boxes en IDs
            aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            // Pose estimation voor elke marker
            vector<Vec3d> rvecs, tvecs;
            aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

            for(size_t i=0; i<markerIds.size(); i++) {
                // Teken de assen van de marker
                // aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength*0.5f);
                cv::drawFrameAxes(frame, cameraMatrix, distCoeffs,
                  rvecs[i], tvecs[i], markerLength*0.5f);

                // Print positie en rotatie
                cout << "Marker ID: " << markerIds[i] 
                     << " tvec: [" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << "]"
                     << " rvec: [" << rvecs[i][0] << ", " << rvecs[i][1] << ", " << rvecs[i][2] << "]" << endl;
            }
        }

        imshow("Camera", frame);

        if(waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}

```

## CameraFrames.cpp: 
Dit script neemt een aantal frames van de camera en slaat ze op als afbeeldingen in plaats van live beeld. Hierdoor kun je zien wat de camera ziet zonder X11 Forwarding. 
```
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0);

    if(!cap.isOpened()) {
        cerr << "Error: Kan camera niet openen" << endl;
        return -1;
    }

    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 3264);
    cap.set(CAP_PROP_FRAME_HEIGHT, 2448);

    Mat frame;
    int frameCount = 0;

    while(true) {
        cap >> frame;
        if(frame.empty()) continue;

        stringstream ss;
        ss << "frames/frame_" << setw(5) << setfill('0') << frameCount << ".jpg";
        imwrite(ss.str(), frame);
        frameCount++;

        if(frameCount >= 10) break; // MAX FRAMES
    }

    cap.release();
    return 0;
}

```

## CMakeLists.txt: 
```
cmake_minimum_required(VERSION 3.10)
project(ArucoCamera)

set(CMAKE_CXX_STANDARD 23)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(DisplayCamera DisplayCamera.cpp)
target_link_libraries(DisplayCamera ${OpenCV_LIBS})

add_executable(Test TestOpenCV.cpp)
target_link_libraries(Test ${OpenCV_LIBS})

add_executable(Frames CameraFrames.cpp)
target_link_libraries(Frames ${OpenCV_LIBS})

add_executable(Markers GenerateMarkers.cpp)
target_link_libraries(Markers ${OpenCV_LIBS})

add_executable(Aruco CameraAruco.cpp)
target_link_libraries(Aruco ${OpenCV_LIBS})

```

## DisplayCamera.cpp:
Een eenvoudig script dat live video van de camera toont. X11 Forwarding nodig!
```
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0, CAP_V4L2);

    if(!cap.isOpened()) {
        cerr << "Error: Kan camera niet openen" << endl;
        return -1;
    }

    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CAP_PROP_FRAME_HEIGHT, 240);

    Mat frame;
    namedWindow("Camera", WINDOW_NORMAL);
    resizeWindow("Camera", 160, 120);

    while(true) {
        cap >> frame;
        if(frame.empty()) continue;

        imshow("Camera", frame);

        if(waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}

```

## GenerateMarkers.cpp:
Dit script genereert PNG-afbeeldingen van ArUco-markers.
```
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

    string savePath = "./markers/";

    for(int id = 0; id <= 5; id++) {  // ID 0 t/m 5
        Mat marker;
        aruco::drawMarker(dictionary, id, 200, marker, 1); // 200x200 pixels
        string filename = savePath + "marker_" + to_string(id) + ".png";
        imwrite(filename, marker);
        cout << "Gegenereerd: " << filename << endl;
    }

    return 0;
}

```

## TestOpenCV.cpp: 
Eenvoudige test of OpenCV correct is geïnstalleerd en kan worden gebruikt.
```
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    return 0;
}

```
## CameraAruco2.cpp: 
Verdere uitwerking van CameraAruco.cpp, versie 2. Geeft de afstand aan en of op die opstand de knop ingedrukt is of niet. Daarnaast is visueel de ID van de markers te zien en niet alleen in de output. 
```
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

int main() {

    VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "Error: Kan camera niet openen" << endl;
        return -1;
    }

    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CAP_PROP_FRAME_HEIGHT, 240);

    Mat frame;
    namedWindow("Camera", WINDOW_NORMAL);
    resizeWindow("Camera", 160, 120);

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::DetectorParameters> parameters =
        aruco::DetectorParameters::create();

    parameters->adaptiveThreshWinSizeMin = 3;
    parameters->adaptiveThreshWinSizeMax = 23;
    parameters->adaptiveThreshWinSizeStep = 2;
    parameters->minMarkerPerimeterRate = 0.03;
    parameters->maxMarkerPerimeterRate = 4.0;
    parameters->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;

    Mat cameraMatrix = (Mat_<double>(3,3) <<
        300, 0, 160,
        0, 300, 120,
        0,   0,   1);

    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);

    float markerLength = 0.044f;

    const double XY_PRESS    = 2.25;
    const double Z_PRESS     = 5.35;
    const double XY_RELEASE  = 2.45;
    const double Z_RELEASE   = 5.60;

    bool knopIngedrukt = false;

    while (true) {

        cap >> frame;
        if (frame.empty()) continue;

        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;
        vector<vector<Point2f>> rejected;

        aruco::detectMarkers(
            frame, dictionary,
            markerCorners, markerIds,
            parameters, rejected
        );

        if (!markerIds.empty()) {

            aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            vector<Vec3d> rvecs, tvecs;
            aruco::estimatePoseSingleMarkers(
                markerCorners,
                markerLength,
                cameraMatrix,
                distCoeffs,
                rvecs,
                tvecs
            );

            for (size_t i = 0; i < markerIds.size(); i++) {

                drawFrameAxes(
                    frame,
                    cameraMatrix,
                    distCoeffs,
                    rvecs[i],
                    tvecs[i],
                    markerLength * 0.5f
                );

                Point2f corner = markerCorners[i][0];
                string text = "ID=" + to_string(markerIds[i]);
                putText(frame, text, corner, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255), 2);

                double x = tvecs[i][0] * 100.0;
                double y = tvecs[i][1] * 100.0;
                double z = tvecs[i][2] * 100.0;

                double xyAfstand = sqrt(x*x + y*y);

                if (!knopIngedrukt) {
                    if (xyAfstand <= XY_PRESS && z <= Z_PRESS) {
                        knopIngedrukt = true;
                    }
                } else {
                    if (xyAfstand > XY_RELEASE || z > Z_RELEASE) {
                        knopIngedrukt = false;
                    }
                }

                cout << "Marker ID: " << markerIds[i] << endl;
                cout << "  Horizontaal : " << abs(x) << " cm "
                     << (x >= 0 ? "rechts" : "links") << endl;
                cout << "  Verticaal   : " << abs(y) << " cm "
                     << (y >= 0 ? "omlaag" : "omhoog") << endl;
                cout << "  Naar voren  : " << z << " cm" << endl;
                cout << "  XY afstand  : " << xyAfstand << " cm" << endl;

                if (knopIngedrukt) {
                    cout << "  >>> KNOP GERAAKT <<<" << endl;
                }

                cout << "----------------------------" << endl;
            }
        }

        imshow("Camera", frame);
        if (waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
```
