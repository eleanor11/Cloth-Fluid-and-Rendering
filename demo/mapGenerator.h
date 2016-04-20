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
		scale = 512;
		init();
	}

	Maps(int s) {
		scale = s;
		init();
	}
	Maps(int s, std::string str) {
		scale = s;
		name = str;

		init();
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

		int sx = x * 2 + int(pos.x) - 1;
		int sy = y * 2 + int(pos.y) - 1;
		//std::cout << x << ' ' << y << ' ' << sx << ' ' << sy << ' ';
		if (sx < 0) sx = 0;
		if (sx >= dx) sx = dx - 1;
		if (sy < 0) sy = 0;
		if (sy >= dy) sy = dy - 1;

		int t = sx * dy + sy;
		//std::cout << mx << ' ' << my << std::endl;
		//std::cout << bmat.channels() << std::endl;
		//std::cout << t << std::endl;
		if (bitMap[t] == 0) {
			saturationRow[t] += s;
			//std::cout << s << std::endl;
			//std::cout << saturationRow[sx * DIMX * 2 + sy] << std::endl;
		}
		else if (bitMap[t] == 255) {
			saturationCol[t] += s;
			//std::cout << saturationCol[sx * DIMX * 2 + sy] << std::endl;
		}
	}

	void renewDiffusing(float kDiffusion, float kDiffusionGravity) {
		Diffusing(kDiffusion, kDiffusionGravity );
	}

private:
	Mat smat, bmat;

	std::vector<float> saturationRow, saturationCol;
	std::vector<int> bitMap;

	int scale;
	int dx, dy;
	int saturationSize;
	int lineWidth = 1;
	std::string name;

	float kDiffusionSide, kDiffusionLevel;

	void init() {
		smat = Mat(scale, scale, CV_8UC1);
		initiateSaturation();
		kDiffusionSide = 0.01;
		kDiffusionLevel = 0.005;
	}

	void initiateSaturation() {
		dx = (DIMX - 1) * 2; 
		dy = (DIMY - 1) * 2;
		saturationSize = dx * dy;
		saturationRow.resize(saturationSize, 0);
		saturationCol.resize(saturationSize, 0);
		
		for (int i = 0; i < smat.rows; i++) {
			for (int j = 0; j < smat.cols; j++) {
				smat.at<uchar>(i, j) = 0;
			}
		}

		imwrite("../../data/textures/s" + name + ".bmp", smat);
	}

	void renewSaturationMap() {
		int idx = 0;
		for (int i = 0; i < dx; i++) {
			for (int j = 0; j < dy; j++) {
				float t = 0;
				if (bitMap[idx] == 0) {
					t = saturationRow[idx];
				}
				else if (bitMap[idx] == 255) {
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

		bitMap.resize(saturationSize, 0);
		int idx = 0;
		for (int i = 0; i < dx; i++) {
			for (int j = 0; j < dy; j++) {
				int bit = (int)(bmat.at<cv::Vec3b>(i * 8 + 4, j * 8 + 4)[0]);
				bitMap[idx++] = bit;
			}
		}
	}

	void renewSaturation(int x, int y, Vec2 pos, float s) {

		int px = (x * 2 + int(pos.x)) * 8;
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

		for (int i = 0; i < saturationSize; i++) {
			float sum = 0;
			float si = saturationRow[i];
			float ds1 = 0, ds2 = 0, ds3 = 0, ds4 = 0, ds5 = 0;

			int x = i / dy;
			int y = i % dy;

			/*TODO: add gravity theta*/
			//row self
			if (y > 0) {
				ds1 = getMax(0.0, kDiffusion * (si - saturationRow[i - 1]));
				sum += ds1;
			}
			if (y < dy - 1) {
				ds2 = getMax(0.0, kDiffusion * (si - saturationRow[i + 1]));
				sum += ds2;
			}
			//row side
			if (x > 0) {
				ds3 = getMax(0.0, kDiffusion * kDiffusionSide * (si - saturationRow[i - dy]));
				sum += ds3;
			}
			if (x < dx - 1) {
				ds4 = getMax(0.0, kDiffusion * kDiffusionSide * (si - saturationRow[i + dy]));
				sum += ds4;
			}
			//row level
			{
				ds5 = getMax(0.0, kDiffusion * kDiffusionLevel * (si - saturationCol[i]));
			}

			float normFac = 1.0;
			if (sum > si) {
				normFac = si / sum;
			}

			if (y > 0) {
				deltasRow[i] += -normFac * ds1;
				deltasRow[i - 1] += normFac * ds1;
			}
			if (y < dy - 1) {
				deltasRow[i] += -normFac * ds2;
				deltasRow[i + 1] += normFac * ds2;
			}
			if (x > 0) {
				deltasRow[i] += -normFac * ds3;
				deltasRow[i - dy] += normFac * ds3;
			}
			if (x < dx - 1) {
				deltasRow[i] += -normFac * ds4;
				deltasRow[i + dy] += normFac * ds4;
			}
			{
				deltasRow[i] += -normFac * ds5;
				deltasCol[i] += normFac * ds5;
			}
		}


		for (int i = 0; i < saturationSize; i++) {
			float sum = 0;
			float si = saturationCol[i];
			float ds1 = 0, ds2 = 0, ds3 = 0, ds4 = 0, ds5 = 0;

			int x = i / dx;
			int y = i % dy;

			/*TODO: add gravity theta*/
			//col self
			if (x > 0) {
				ds1 = getMax(0.0, kDiffusion * (si - saturationCol[i - dy]));
				sum += ds1;
			}
			if (x < dx - 1) {
				ds2 = getMax(0.0, kDiffusion * (si - saturationCol[i + dy]));
				sum += ds2;
			}
			//col side
			if (y > 0) {
				ds3 = getMax(0.0, kDiffusion * kDiffusionSide * (si - saturationCol[i - 1]));
				sum += ds3;
			}
			if (y < dy - 1) {
				ds4 = getMax(0.0, kDiffusion * kDiffusionSide * (si - saturationCol[i + 1]));
				sum += ds4;
			}
			//col level
			{
				ds5 = getMax(0.0, kDiffusion * kDiffusionLevel * (si - saturationRow[i]));
				sum += ds5;
			}

			float normFac = 1.0;

			if (sum > si) {
				normFac = si / sum;
			}

			if (x > 0) {
				deltasCol[i] += -normFac * ds1;
				deltasCol[i - dy] += normFac * ds1;
			}
			if (x < dx - 1) {
				deltasCol[i] += -normFac * ds2;
				deltasCol[i + dy] += normFac * ds2;
			}
			if (y > 0) {
				deltasCol[i] += -normFac * ds3;
				deltasCol[i - 1] += normFac * ds3;
			}
			if (y < dy - 1) {
				deltasCol[i] += -normFac * ds4;
				deltasCol[i + 1] += normFac * ds4;
			}
			{
				deltasCol[i] += -normFac * ds5;
				deltasRow[i] += normFac * ds5;
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