#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <stdio.h>
const int dot_ = 16;

using namespace std;
using namespace cv;

struct dot {
	double x;//column
	double y;//row
	bool n = false;//false 时检测不到黑像素
};
struct k_ {
	double k = 0;
	bool p = false;
};

//double offset_k = 3;//弯道斜率差判断标准
int mode = 1;//0：自动控制转弯模式；1：直行调整同时检测弯道模式；
double k_offset = 0.2;
double k_turn = 4;
int k_turn_ = 45;
double k_branch = 4;
double x_trans = 0.02;
double y_trans;
double scale;
int count_turn = 0;


void send_offset_k(double k) {
	cout << "k=" << k << endl;
}

void turn(double x, double y) {
	cout << "turn: " << x << " , " << y << endl;
	//透视变换
	int pixel_x_bef = 960 - x;
	int pixel_y = pixel_x_bef * y_trans;
	int pixel_x = pixel_x_bef * x_trans + (y - 640);
	double x_real = pixel_x * scale;
	double y_real = pixel_y * scale;
	count_turn++;
	//mode = 0;
	//
	/*if (count_turn == 3){
	cout << "To turn" << endl;
	}*/
}

void main()
{
	std::vector<Point2f> bef_dot(4);
	std::vector<Point2f> now_dot(4);
	bef_dot[0] = Point2f(911, 593);
	bef_dot[1] = Point2f(1032, 593);
	bef_dot[2] = Point2f(916, 514);
	bef_dot[3] = Point2f(1013, 514);
	now_dot[0] = Point2f(964, 1440);
	now_dot[1] = Point2f(1010, 1440);
	now_dot[2] = Point2f(1010, 1300);
	now_dot[3] = Point2f(900, 1300);

	Mat trans_image = getPerspectiveTransform(bef_dot, now_dot);
	Mat image_trans;
	// warpPerspective(image, image_trans, trans_image,image_trans.size());
	vector<Point2f> dot_test(1);
	dot_test[0] = cvPoint(300, 300);
	vector<Point2f> dot_test_trans;
	//Mat image_trans;
	InputArray dotaaa(dot_test);
	Mat aaa = dotaaa.getMat();
	try {
		perspectiveTransform(dotaaa, dot_test_trans, trans_image);
	}
	catch (exception& e) {
		cout << e.what() << endl;
	}

	// 获取视频文件
	VideoCapture capture("all.avi");
	Mat img;
	while (1) {

		dot dots[dot_];//检测点
		k_ ks[dot_ - 1];//斜率
		bool if_null = true;//此帧图片是否为空
		bool if_offset = false;//此帧小车是否需要偏转

		capture >> img;
		if (img.empty())
			break;

		Mat img_gray;
		cvtColor(img, img_gray, CV_BGR2GRAY);

		Mat img_binary;
		threshold(img_gray, img_binary, 150, 255, CV_THRESH_BINARY);

		Mat &image = img_binary;
		bitwise_not(image, image);
		Mat Imglabels, Imgstats, Imgcentriods;
		int Imglabelnum = connectedComponentsWithStats(image, Imglabels, Imgstats, Imgcentriods);
		cout << Imglabelnum << endl << image.rows << "," << image.cols << " " << Imglabels.rows << "," << Imglabels.cols << endl;

		int max_conn_label = 1;
		for (int i = 2; i < Imglabelnum; i++) {//求最大连通块的label
			long* max_conn = Imgstats.ptr<long>(max_conn_label);
			long* max_conn_i = Imgstats.ptr<long>(i);
			//cout <<max_conn_i[4]<< endl;
			if (max_conn[4]<max_conn_i[4]) {
				max_conn_label = i;
			}
		}
		double k_bef = 0;//前面点的平均斜率
		int offset_dot = 10;//偏移点的下标
		int rows_now = Imglabels.rows * 2 / 5;//目前行数,ALL=1440,1920
		for (int i = 0; i < dot_; i++) {//

			int rows_c = 0;
			int cols_c = 0;
			int num = 0;

			for (int j = 0; j < Imglabels.rows / 2 / dot_; j++) {
				int* label_row = Imglabels.ptr<int>(rows_now);
				for (int k = 0; k < Imglabels.cols; k++) {//each row
					if (label_row[k] == max_conn_label) {
						num++;
						cols_c += k;
						rows_c += rows_now;
						//cout << k << " , " << rows_now << endl;
					}
				}
				//cout << endl;
				rows_now++;

			}

			if (num == 0)//没有属于最大连通块的像素
				dots[i].n = false;
			else {
				if_null = false;
				dots[i].n = true;
				dots[i].y = cols_c / num;
				dots[i].x = rows_c / num;
				cout << dots[i].x << " , " << dots[i].y << endl;
				uchar* data2 = image.ptr<uchar>(dots[i].x);
				for (int ii = 0; ii < 10; ii++) {
					data2[int(dots[i].y + ii)] = 150;//画出中心点
				}
				//}

			}

		}

		for (int i = dot_ - 1; i >= 1; i--) {//算15个斜率
			if (dots[i].n == true && dots[i - 1].n == true) {
				double get_k = (dots[i].y - dots[i - 1].y) / (dots[i].x - dots[i - 1].x);//斜率
				ks[i - 1].k = get_k;
				ks[i - 1].p = true;
				cout << get_k << endl;
			}
		}

		if (mode == 1) {
			int count_offset = 0;
			double all_k = 0;
			for (int i = dot_ - 2; i>dot_ - 8; i--) {//检测偏移量
				if (ks[i].p == true && fabs(ks[i].k) > k_offset) {
					if_offset = true;
				}
			}
			if (if_offset == true) {
				for (int i = dot_ - 2; i>dot_ - 8; i--) {//计算偏移量

					if (ks[i].p == true) {
						all_k += ks[i].k;
						count_offset++;
					}
				}
				if (if_offset)
					send_offset_k(all_k / count_offset);
			}


			if (ks[dot_ - 2].p == true) {
				double pre_k = ks[dot_ - 2].k;
				for (int i = dot_ - 2; i >= 0; i--) {
					if (pre_k == 0) {
						if (ks[i].p == true && fabs(ks[i].k - pre_k) > k_turn)
							turn(dots[i].x, dots[i].y);//定位弯道
						else if (ks[i].p == false)
							break;
					}
					else {
						if (ks[i].p == true && ((fabs(ks[i].k) / fabs(pre_k) > k_turn_) || (ks[i].k*pre_k<0 && fabs(ks[i].k - pre_k)>2 * k_branch))) {
							cout << i << endl;
							turn(dots[i].x, dots[i].y);//定位弯道
						}
						else if (ks[i].p == false)
							break;
					}
					pre_k = ks[i].k;
				}
			}


		}
		else if (mode == 0) {

		}

		imshow("", image);
		waitKey(0);
		//system("pause");
	}
}
