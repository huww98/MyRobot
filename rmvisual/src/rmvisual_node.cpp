#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <rmvisual/turn.h>
#include <rmvisual/visual_info.h>

const int dot_ = 16;

using namespace std;
using namespace cv;

struct dot
{
    double x;       //column
    double y;       //row
    bool n = false; //false 时检测不到黑像素
};
struct k_
{
    double k = 0;
    bool p = false;
};

//double offset_k = 3;//弯道斜率差判断标准
int mode = 1; //0：自动控制转弯模式；1：直行调整同时检测弯道模式；
double k_offset = 0.2;
double k_turn = 2.5;
int k_turn_ = 45;
double k_branch = 1.1;
double x_trans = 0.02;
double y_trans;
double scale;
int count_turn = 0;

ros::Publisher turnPub;
ros::Publisher visualInfoPub;

void send_offset_k(double k, double x_offset)
{
    rmvisual::visual_info msg;
    msg.header.stamp = ros::Time::now();
    msg.k = -k;
    msg.x_offset = x_offset;
    visualInfoPub.publish(msg);
}

void turn(double x, double y, double k)
{
    rmvisual::turn msg;
    msg.header.stamp = ros::Time::now();
    msg.distance = sqrt(x*x+y*y);
    msg.k = -k;
    turnPub.publish(msg);
}

void view(Mat img)
{
    dot dots[dot_];         //检测点
    k_ ks[dot_ - 1];        //斜率
    bool if_null = true;    //此帧图片是否为空
    bool if_offset = false; //此帧小车是否需要偏转

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

    Mat img_binary;
    threshold(img_gray, img_binary, 150, 255, CV_THRESH_BINARY);

    Mat &image = img_binary;
    bitwise_not(image, image);
    Mat Imglabels, Imgstats, Imgcentriods;
    int Imglabelnum = connectedComponentsWithStats(image, Imglabels, Imgstats, Imgcentriods);
    for (int i = 2; i < Imglabelnum; i++)
    { //求最大连通块的label
        long *max_conn = Imgstats.ptr<long>(max_conn_label);
        long *max_conn_i = Imgstats.ptr<long>(i);
        //cout <<max_conn_i[4]<< endl;
        if (max_conn[4] < max_conn_i[4])
        {
            max_conn_label = i;
        }
    }

    double k_bef = 0;                      //前面点的平均斜率
    int offset_dot = 10;                   //偏移点的下标
    int rows_now = Imglabels.rows * 2 / 5; //目前行数,ALL=1440,1920
    for (int i = 0; i < dot_; i++)
    {

        int rows_c = 0;
        int cols_c = 0;
        int num = 0;

        for (int j = 0; j < Imglabels.rows / 2 / dot_; j++)
        {
            int *label_row = Imglabels.ptr<int>(rows_now);
            for (int k = 0; k < Imglabels.cols; k++)
            { //each row
                if (label_row[k] == max_conn_label)
                {
                    num++;
                    cols_c += k;
                    rows_c += rows_now;
                    //cout << k << " , " << rows_now << endl;
                }
            }
            //cout << endl;
            rows_now++;
        }

        if (num == 0) //没有属于最大连通块的像素
            dots[i].n = false;
        else
        {
            if_null = false;
            dots[i].n = true;
            dots[i].y = cols_c / num;
            dots[i].x = rows_c / num;
            cout << dots[i].x << " , " << dots[i].y << endl;

            uchar *data2 = image.ptr<uchar>(dots[i].x);
            for (int ii = 0; ii < 10; ii++)
            {
                data2[int(dots[i].y + ii)] = 150; //画出中心点
            }
        }
    }
    vector<Point2d> dot_vec(dot_);
    for (int i = 0; i < dot_; i++)
    {
        if (dots[i].n == true)
        {
            dot_vec[i].x = dots[i].x;
            dot_vec[i].y = dots[i].y;
        }
    }

    vector<Point2d> dots_trans;

    perspectiveTransform(dot_vec, dots_trans, trans_image);

    for (int i = 0; i < dot_; i++)
    {
        if (dots[i].n == true)
        {
            cout << dots_trans[i].x << "," << dots_trans[i].y << endl;
        }
    }

    double first_x;
    for (int i = 0; i < dot_; i++)
    {
        if (dots[i].n == true)
        {
            first_x = dots[i].y;
            break;
        }
    }

    for (int i = dot_ - 1; i >= 1; i--)
    { //算15个斜率
        if (dots[i].n == true && dots[i - 1].n == true)
        {
            double get_k = (dots_trans[i].y - dots_trans[i - 1].y) / (dots_trans[i].x - dots_trans[i - 1].x); //斜率
            ks[i - 1].k = get_k;
            ks[i - 1].p = true;
            cout << get_k << endl;
        }
    }

    int count_offset = 0;
    double all_k = 0;
    double sent_k;

    for (int i = dot_ - 2; i > dot_ - 8; i--)
    { //检测偏移量
        if (ks[i].p == true)
        {
            if_offset = true;
        }
    }
    if (if_offset == true)
    {
        for (int i = dot_ - 2; i > dot_ - 8; i--)
        { //计算偏移量

            if (ks[i].p == true)
            {
                all_k += ks[i].k;
                count_offset++;
            }
        }

        sent_k = all_k / count_offset;
        send_offset_k(sent_k, first_x);
    }

    if (ks[dot_ - 2].p == true)
    {
        double pre_k = ks[dot_ - 2].k;
        for (int i = dot_ - 2; i >= 0; i--)
        {
            if (fabs(pre_k) < 0.3)
            {
                if (ks[i].p == true && fabs(ks[i].k - pre_k) > k_turn)
                    turn(dots_trans[i].x, dots_trans[i].y, sent_k); //定位弯道
                else if (ks[i].p == false)
                    break;
            }
            else
            {
                if (ks[i].p == true && ((fabs(ks[i].k) / fabs(pre_k) > k_turn_) || (ks[i].k * pre_k < 0 && fabs(ks[i].k - pre_k) > 2 * k_branch)))
                {
                    cout << i << endl;
                    turn(dots_trans[i].x, dots_trans[i].y, sent_k); //定位弯道
                }
                else if (ks[i].p == false)
                    break;
            }
            pre_k = ks[i].k;
        }
    }
    if (ks[dot_ - 2].p)
    {
        bool if_straight = true; //检测出口或标准的丁字路口
        int i_;
        for (int i = dot_ - 2; i >= 0; i--)
        {
            if (if_straight)
            {
                if (ks[i].p)
                {
                    if (fabs(ks[i].k) < k_offset)
                    {
                        if (i == 0)
                            if_straight = false; //全直
                        else
                            continue;
                    }
                    else
                        if_straight = false;
                }
                else
                {
                    i_ = i + 1;
                    break;
                }
            }
            else
                break;
        }
        if (if_straight)
        {
            turn(dots[i_].x, dots[i_].y, sent_k); //最后那个点
        }
    }

    namedWindow("", WINDOW_NORMAL);
    imshow("", image);
    waitKey(0);
    //system("pause");
}

void img_cb(const sensor_msgs::ImageConstPtr& msg)
{
    auto cv_ptr = cv_bridge::toCvCopy(msg);
    view(cv_ptr->image);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmvisual");
    ros::NodeHandle nh;

    auto image_sub = nh.subscribe("image", 1, &img_cb);
    turnPub = nh.advertise<rmvisual::turn>("turn", 2);
    visualInfoPub = nh.advertise<rmvisual::visual_info>("visual_info", 2);
}
