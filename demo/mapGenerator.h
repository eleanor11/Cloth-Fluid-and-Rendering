#include <opencv.hpp>
#include <core\core.hpp>
#include <highgui\highgui.hpp>

using namespace cv;

const float MAXSATURATION = 10.0;
const int DIMX = 32;
const int DIMY = 32;

const int WIDTHBASE = 8;

class Maps {

private:
	Mat smat;

	int scale = 512;
	int lineWidth = 1;
	std::string name;

	void initiateSaturation() {
		for (int i = 0; i < smat.rows; i++) {
			for (int j = 0; j < smat.cols; j++) {
				smat.at<uchar>(i, j) = 0;
			}
		}
	}

public:
	Maps() {
		smat = Mat(scale, scale, CV_8UC1);
	}

	Maps(int s) {
		scale = s;
		smat = Mat(scale, scale, CV_8UC1);
		initiateSaturation();
	}
	Maps(int s, std::string str) {
		scale = s;
		name = str;
		smat = Mat(scale, scale, CV_8UC1);
		initiateSaturation();
	}

	void setScale(int s) {
		scale = s;
	}
	int getScale() {
		return scale;
	}
	void setLineWidth(int l) {
		lineWidth = l;
	}
	int getLineWidth() {
		return lineWidth;
	}
	void setName(std::string str) {
		name = str;
	}
	std::string getName() {
		return name;
	}

	void writeToFile() {
		imwrite("../../data/textures/s" + name + ".bmp", smat);
		//std::cout << "file" << std::endl;
	}

	void renewSaturation(int x, int y, Vec2 pos, float s) {

		int px = ((DIMX - x) * 2 + int(1 - pos.x)) * 8;
		int py = (y * 2 + int(pos.y)) * 8;

		int t = smat.at<uchar>(px, py);
		t += (int)(s / MAXSATURATION * 255);
		if (t > 255) t = 255;

		for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 8; j++) {
				smat.at<uchar>(px + i, py + j) = t;
			}
		}

		//std::cout << px << ' ' << py << ' ' << t << std::endl;
	}
};