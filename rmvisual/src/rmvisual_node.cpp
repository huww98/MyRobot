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
	bool n = false;//false ʱ��ⲻ��������
};
struct k_ {
	double k = 0;
	bool p = false;
};

//double offset_k = 3;//���б�ʲ��жϱ�׼
int mode = 1;//0���Զ�����ת��ģʽ��1��ֱ�е���ͬʱ������ģʽ��
double k_offset = 0.2;
double k_turn = 2.5;
int k_turn_ = 45;
double k_branch = 1.1;
double x_trans = 0.02;
double y_trans;
double scale;
int count_turn = 0;


void send_offset_k(double k) {
	cout << "k=" << k << endl;
}

void turn(double x, double y) {
	cout << "turn: " << x << " , " << y << endl;
	
}
void view(Mat img) {
	dot dots[dot_];//����
	k_ ks[dot_ - 1];//б��
	bool if_null = true;//��֡ͼƬ�Ƿ�Ϊ��
	bool if_offset = false;//��֡С���Ƿ���Ҫƫת
						   //img = cvQueryFrame(capture);

	std::vector<Point2f> bef_dot(4);
	std::vector<Point2f> now_dot(4);
	bef_dot[0] = Point2f(915, 0);
	bef_dot[1] = Point2f(915, 1280);
	bef_dot[2] = Point2f(370, 900);
	bef_dot[3] = Point2f(370, 380);
	now_dot[0] = Point2f(170, -105);
	now_dot[1] = Point2f(170, 105);
	now_dot[2] = Point2f(467, 105);
	now_dot[3] = Point2f(467, -105);
	
	Mat trans_image = getPerspectiveTransform(bef_dot, now_dot);
	int max_conn_label = 1;

	Mat img_gray;
	cvtColor(img, img_gray, CV_BGR2GRAY);
	//imwrite("img_gray.jpg", img_gray);

	Mat img_binary;
	threshold(img_gray, img_binary, 150, 255, CV_THRESH_BINARY);
	//imwrite("img_binary.jpg", img_binary);

	Mat &image = img_binary;
	bitwise_not(image, image);
	Mat Imglabels, Imgstats, Imgcentriods;
	int Imglabelnum = connectedComponentsWithStats(image, Imglabels, Imgstats, Imgcentriods);
	for (int i = 2; i < Imglabelnum; i++) {//�������ͨ���label
		long* max_conn = Imgstats.ptr<long>(max_conn_label);
		long* max_conn_i = Imgstats.ptr<long>(i);
		//cout <<max_conn_i[4]<< endl;
		if (max_conn[4]<max_conn_i[4]) {
			max_conn_label = i;
		}
	}

	double k_bef = 0;//ǰ����ƽ��б��
	int offset_dot = 10;//ƫ�Ƶ���±�
	int rows_now = Imglabels.rows * 2 / 5;//Ŀǰ����,ALL=1440,1920
	for (int i = 0; i < dot_; i++) {//

		int rows_c = 0;
		int cols_c = 0;
		int num = 0;
		//cv::Vec3b * data_ = image_color.ptr<cv::Vec3b>(rows_now);
		//if (i == 0){
		//	for (int i = 0; i < 96; i++){
		//		uchar* data = image.ptr(rows_now);
		//		for (int k = 0; k < 3840; k++){//each row
		//			//cout << int(image.at<uchar>(rows_now,k)) << " ";
		//			if (int(data[k]) < 10){
		//				num++;
		//				cols_c += k;
		//				rows_c += rows_now;
		//				//cout << k << " , " << rows_now << endl;
		//			}
		//		}
		//		//cout << endl;
		//		rows_now++;
		//	}
		//}
		//else

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

		//for (int j = 0; j < 32; j++){
		//	uchar* label_row = Imglabels.ptr(rows_now);
		//	uchar* data = image.ptr(rows_now);
		//	for (int k = 0; k < 1280; k++){//each row
		//		if (int(data[k]) < 10){
		//			num++;
		//			cols_c += k;
		//			rows_c += rows_now;
		//			cout << int(label_row[k]) << ",";
		//			//cout << k << " , " << rows_now << endl;
		//		}
		//	}
		//	//cout << endl;
		//	rows_now++;

		//}

		if (num == 0)//û�����������ͨ�������
			dots[i].n = false;
		else {
			if_null = false;
			dots[i].n = true;
			dots[i].y = cols_c / num;
			dots[i].x = rows_c / num;
			cout << dots[i].x << " , " << dots[i].y << endl;

			uchar* data2 = image.ptr<uchar>(dots[i].x);
			for (int ii = 0; ii < 10; ii++) {
				data2[int(dots[i].y + ii)] = 150;//�������ĵ�
			}
			//}

		}

	}
	vector<Point2d> dot_vec(dot_);
	for (int i = 0; i < dot_; i++) {
		if (dots[i].n == true) {
			dot_vec[i].x = dots[i].x;
			dot_vec[i].y = dots[i].y;
		}

	}
	//warpPerspective(image, image_trans, trans_image, image_trans.size());

	vector<Point2d> dots_trans;
	//Mat image_trans;
	//InputArray dotaaa(dot_test);
	//Mat aaa = dotaaa.getMat();
	//try {
	perspectiveTransform(dot_vec, dots_trans, trans_image);
	//if (if_null){
	//	break;//����֡Ϊ����ֹͣ
	//}
	for (int i = 0; i < dot_; i++) {
		if (dots[i].n == true) {
			cout << dots_trans[i].x << "," << dots_trans[i].y << endl;


		}
	}


	for (int i = dot_ - 1; i >= 1; i--) {//��15��б��
		if (dots[i].n == true && dots[i - 1].n == true) {
			double get_k = (dots_trans[i].y - dots_trans[i - 1].y) / (dots_trans[i].x - dots_trans[i - 1].x);//б��
			ks[i - 1].k = get_k;
			ks[i - 1].p = true;
			cout << get_k << endl;
			/*if (fabs(get_k - k_bef) >= offset_k){
			offset_dot = i - 1;
			cout << offset_dot + 1 << " is a offset dot" << endl;
			}
			else
			k_bef = (k_bef + get_k) / 2;*/
		}
	}

	//if (ks[0].p == true && ks[0].k > offset_k){//��λ���λ��

	//}



	if (mode == 1) {
		int count_offset = 0;
		double all_k = 0;
		for (int i = dot_ - 2; i>dot_ - 8; i--) {//���ƫ����
			if (ks[i].p == true && fabs(ks[i].k) > k_offset) {
				if_offset = true;
			}
		}
		if (if_offset == true) {
			for (int i = dot_ - 2; i>dot_ - 8; i--) {//����ƫ����

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
				if (fabs(pre_k) < 0.3) {
					if (ks[i].p == true && fabs(ks[i].k - pre_k) > k_turn)
						turn(dots_trans[i].x, dots_trans[i].y);//��λ���
					else if (ks[i].p == false)
						break;
				}
				else {
					if (ks[i].p == true && ((fabs(ks[i].k) / fabs(pre_k) > k_turn_) || (ks[i].k*pre_k<0 && fabs(ks[i].k - pre_k)>2 * k_branch))) {
						cout << i << endl;
						turn(dots_trans[i].x, dots_trans[i].y);//��λ���
					}
					else if (ks[i].p == false)
						break;
				}
				pre_k = ks[i].k;
			}
		}
		if (ks[dot_ - 2].p) {
			bool if_straight = true;//�����ڻ��׼�Ķ���·��
			int i_;
			for (int i = dot_ - 2; i >= 0; i--) {
				if (if_straight) {
					if (ks[i].p) {
						if (fabs(ks[i].k) < k_offset) {
							if (i == 0)
								if_straight = false;//ȫֱ
							else
								continue;
						}
						else
							if_straight = false;
					}
					else {
						i_ = i+1;
						break;
					}
				}
				else
					break;
			}
			if (if_straight) {
				turn(dots[i_].x, dots[i_].y);//����Ǹ���
			}
		}

	}
	else if (mode == 0) {

	}

	namedWindow("", WINDOW_NORMAL);
	imshow("", image);
	waitKey(0);
	//system("pause");
}
void main()
{
	

	// ��ȡ��Ƶ�ļ�
	VideoCapture capture("all3.avi");
	//int frameH = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
	//int frameW = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
	//int fps = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
	//int numFrames = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT);

	//IplImage* img = 0;
	//int i = 0;
	//cout << numFrames << endl;
	

	Mat img;
	while (1) {
	
	capture >> img;
	if (img.empty())
		break;
	view(img);
		
	}
}

