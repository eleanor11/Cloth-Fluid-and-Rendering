#include <opencv.hpp>
#include <core\core.hpp>
#include <highgui\highgui.hpp>

using namespace cv;

const float MAXSATURATION = 10.0;
const int DIMX = 32;
const int DIMY = 32;

const int WIDTHBASE = 8;

float getMax(float x, float y) {
	if (x > y) return x;
	else return y;
}
float getMin(float x, float y) {
	if (x < y) return x;
	else return y;
}

class Maps {
public:
	Maps() {
		smat = Mat(scale, scale, CV_8UC1);
	}

	Maps(int s) {
		scale = s;
		smat = Mat(scale, scale, CV_8UC1);
		saturationSize = s * s / 64;
		initiateSaturation();
	}
	Maps(int s, std::string str) {
		scale = s;
		name = str;
		smat = Mat(scale, scale, CV_8UC1);
		saturationSize = s * s / 64;
		initiateSaturation();
		readBmap();
	}

	void setScale(int s) {
		scale = s;
		saturationSize = s * s / 64;
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
		renewSaturationMap();
		imwrite("../../data/textures/s" + name + ".bmp", smat);
		//std::cout << "file" << std::endl;
	}


	void renewAbsorbing(int x, int y, Vec2 pos, float s) {

		int sx = (DIMX - x - 1) * 2 + int(1 - pos.x);
		int sy = y * 2 + int(pos.y);
		int mx = sx * 8;
		int my = sy * 8;

		int bit = (int)(bmat.at<cv::Vec3b>(mx + 4, my + 4)[0]);
		//std::cout << mx << ' ' << my << std::endl;
		//std::cout << bmat.channels() << std::endl;
		if (bit == 0) {
			saturationRow[sx * DIMX * 2 + sy] += s;
			//std::cout << s << std::endl;
			//std::cout << saturationRow[sx * DIMX * 2 + sy] << std::endl;
		}
		else if (bit == 255) {
			saturationCol[sx * DIMX * 2 + sy] += s;
			//std::cout << saturationCol[sx * DIMX * 2 + sy] << std::endl;
		}
	}

	void renewDiffusing(float kDiffusion, float kDiffusionGravity) {
		Diffusing(kDiffusion, kDiffusionGravity );
	}

private:
	Mat smat, bmat;

	std::vector<float> saturationRow, saturationCol;

	int scale = 512;
	int saturationSize;
	int lineWidth = 1;
	std::string name;

	void initiateSaturation() {
		saturationRow.resize(saturationSize, 0);
		saturationCol.resize(saturationSize, 0);
		
		for (int i = 0; i < smat.rows; i++) {
			for (int j = 0; j < smat.cols; j++) {
				smat.at<uchar>(i, j) = 0;
			}
		}
	}

	void renewSaturationMap() {
		int idx = 0;
		for (int i = 0; i < DIMX * 2; i++) {
			for (int j = 0; j < DIMY * 2; j++) {
				int bit = (int)(bmat.at<cv::Vec3b>(i * 8, j * 8)[0]);
				float t = 0;
				if (bit == 0) {
					t = saturationRow[idx];
				}
				else if (bit == 255) {
					t = saturationCol[idx];
				}
				idx++;
				t = t / MAXSATURATION * 255;
				if (t > 255) t = 255;

				for (int ii = 0; ii < 8; ii++) {
					for (int jj = 0; jj < 8; jj++) {
						smat.at<uchar>(i * 8 + ii, j * 8 + jj) = t;
					}
				}
			}
		}
	}

	void readBmap() {
		bmat = imread("../../data/textures/b" + name + ".bmp");
	}

	void renewSaturation(int x, int y, Vec2 pos, float s) {

		int px = ((DIMX - x - 1) * 2 + int(1 - pos.x)) * 8;
		int py = (y * 2 + int(pos.y)) * 8;

		int t = smat.at<uchar>(px, py);
		t += (int)(s / MAXSATURATION * 255);
		if (t > 255) t = 255;

		for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 8; j++) {
				smat.at<uchar>(px + i, py + j) = t;
			}
		}
	}

	void Diffusing(float kDiffusion, float kDiffusionGravity) {
		std::vector<float> deltasRow;
		std::vector<float> deltasCol;
		deltasRow.resize(saturationSize, 0);
		deltasCol.resize(saturationSize, 0);

		//row self
		for (int i = 0; i < saturationSize; i++) {
			float sum = 0;
			float si = saturationRow[i];
			float ds1 = 0, ds2 = 0;

			int x = i / (DIMX * 2);
			int y = i % (DIMX * 2);

			/*TODO: add gravity theta*/
			if (y > 0) {
				ds1 = getMax(0.0, kDiffusion * (si - saturationRow[i - 1]));
				sum += ds1;
			}
			if (y < DIMX * 2 - 1) {
				ds2 = getMax(0.0, kDiffusion * (si - saturationRow[i + 1]));
				sum += ds2;
			}
			float normFac = 1.0;
			if (sum > si) {
				normFac = si / sum;
			}

			if (y > 0) {
				deltasRow[i] += -normFac * ds1;
				deltasRow[i - 1] += normFac * ds1;
			}
			if (y < DIMX * 2 - 1) {
				deltasRow[i] += -normFac * ds2;
				deltasRow[i + 1] += normFac * ds2;
			}
		}

		// col self
		for (int i = 0; i < saturationSize; i++) {
			float sum = 0;
			float si = saturationCol[i];
			float ds1 = 0, ds2 = 0;

			int x = i / (DIMX * 2);
			int y = i % (DIMX * 2);

			/*TODO: add gravity theta*/
			if (x > 0) {
				ds1 = getMax(0.0, kDiffusion * (si - saturationCol[i - DIMX * 2]));
				sum += ds1;
			}
			if (x < DIMX * 2 - 1) {
				ds2 = getMax(0.0, kDiffusion * (si - saturationCol[i + DIMX * 2]));
				sum += ds2;
			}
			float normFac = 1.0;
			if (sum > si) {
				normFac = si / sum;
			}

			if (x > 0) {
				deltasCol[i] += -normFac * ds1;
				deltasCol[i - DIMX * 2] += normFac * ds1;
			}
			if (x < DIMX * 2 - 1) {
				deltasCol[i] += -normFac * ds2;
				deltasCol[i + DIMX * 2] += normFac * ds2;
			}
		}


		//renew
		for (int i = 0; i < saturationSize; i++) {
			saturationRow[i] += deltasRow[i];
			if (saturationRow[i] < 0) {
				saturationRow[i] = 0;
			}

			saturationCol[i] += deltasCol[i];
			if (saturationCol[i] < 0) {
				saturationCol[i] = 0;
			}
		}

	}

};