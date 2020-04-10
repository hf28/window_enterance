#include <ros/ros.h>
#include <image_transport/image_transport.h>   //includes everything we need to publish and subscribe to images.
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "simulate/imtodyn.h"
#include <algorithm>
using namespace cv;
using namespace std;

const int max_canny1 = 800;
const int max_canny2 = 800;
const int max_GuKernelSize = 50;
const float shapeAR = 1.5;
const String window_capture_name = "Video Capture";
const int max_value_H = 360/2;
const int max_value = 255;
int low_H = 15, low_S = 174, low_V = 0;
int high_H = 23, high_S = 255, high_V = 255;
int canny1 = 131, canny2 = 0, GuKernelSize = 7;
float GuSigma = 1.2;
int vecy, vecz;
bool flag = false, msgFlag = false;

static void on_canny1_trackbar(int, void *)
{
    // low_H = min(high_H-1, low_H);
    setTrackbarPos("canny1", window_capture_name, canny1);
}

static void on_canny2_trackbar(int, void *)
{
    // low_H = min(high_H-1, low_H);
    setTrackbarPos("canny1", window_capture_name, canny1);
}

static void on_GuKernelSize_trackbar(int, void *)
{
    if(GuKernelSize%2 == 0) GuKernelSize += 1;
    setTrackbarPos("GuKernelSize", window_capture_name, GuKernelSize);
}

static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_capture_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_capture_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_capture_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_capture_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_capture_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_capture_name, high_V);
}

float arCalculate(vector<Point>, Mat);

float euclideanDist(Point, Point);

bool csort (vector<Point>, vector<Point2d>&);

Mat imageProcessing(Mat);

void rectangleGeometric(vector<Point>,  Mat, int&, int&);

bool sortcol( const vector<float>& v1, const vector<float>& v2 ) {
 return v1[1] < v2[1];
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)    //will get called when a new image has arrived on the
{      //"camera/image" topic. Although the image may have been sent in some arbitrary transport-specific message type,
  cout<<"Iteration Begin\n\n\n";
  msgFlag = true;
  try  // notice that the callback need only handle the normal sensor_msgs/Image type. All image encoding/decoding is
  {    //handled automagically
    // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);  //We convert the ROS image message to an OpenCV image
                                                                   //with BGR pixel encoding, then show it in a display window.
    Mat frame, frm, img = cv_bridge::toCvShare(msg, "bgr8")->image;
    // imshow("token pic", img);
    frm = img.clone();
    frame = imageProcessing(frm);
    // imshow("processed pic", frame);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "impro");
  ros::NodeHandle nh;
  // cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/quadrotor_1/front/image_raw", 1, imageCallback);
  ros::Publisher pubinfo = nh.advertise<simulate::imtodyn>("visual_info", 10);
  ros::Rate loop_rate(10); // Loop at 10Hz
  simulate::imtodyn msg;

  // msg.header.stamp = ros::Time::now();
  // msg.header.frame_id = "/world";


  int count = 0;
  while (ros::ok())
  {
    if(count == 0) cout<<"Node Started\n";
    // if(!msgFlag) cout<<"No Input\n";
    // cout<<"main\t"<<vecy<<'\t'<<vecz<<endl;
    // cout<<"\n\n\n----------------------"/*<<\nimpro flag:\t"<<flag*/<<endl;
    if(flag){
      msg.y = vecy;
      msg.z = vecz;
      cout<<"impro published data\tmsg.y:"<<msg.y<<"\tmsg.z:\t"<<msg.z<<endl;
      pubinfo.publish(msg);
      flag = false;
    }
    else cout<<"nothing published\n";
    cout<<"\n\n\nIteration End\n----------------------\n";
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    msgFlag = false;
  }
  // cv::destroyWindow("view");
  return 0;
}

Mat imageProcessing(Mat imin){

  cout<<"\n~~~~~\nIP is called\n";

   namedWindow(window_capture_name);
   createTrackbar("canny1", window_capture_name, &canny1, max_canny1, on_canny1_trackbar);
   createTrackbar("canny2", window_capture_name, &canny2, max_canny2, on_canny2_trackbar);
   createTrackbar("GuKernelSize", window_capture_name, &GuKernelSize, max_GuKernelSize, on_GuKernelSize_trackbar);
   // createTrackbar("canny2", window_capture_name, &canny2, max_canny2, on_canny2_trackbar);
   createTrackbar("Low H", window_capture_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
   createTrackbar("High H", window_capture_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
   createTrackbar("Low S", window_capture_name, &low_S, max_value, on_low_S_thresh_trackbar);
   createTrackbar("High S", window_capture_name, &high_S, max_value, on_high_S_thresh_trackbar);
   createTrackbar("Low V", window_capture_name, &low_V, max_value, on_low_V_thresh_trackbar);
   createTrackbar("High V", window_capture_name, &high_V, max_value, on_high_V_thresh_trackbar);

   int goodIndex = 0, ind = 0;
   bool four_found = false;
   float ar;// arDifference = 10e6;
   RNG rng(12345);
   Mat imin_hsv, imout, drawing;
   vector<vector<Point> > contours, poly;
   vector<vector<float> > arDifference;
   vector<Vec4i> hierarchy;
   vector<Point> conto;
   vector<Point2d> contor;
   int erosion_size = 3;
   Mat element = getStructuringElement( MORPH_RECT,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

   // cvtColor(imin, mask, CV_BGR2GRAY);
   drawing = imin.clone();
   // GaussianBlur(imin, imin, Size(GuKernelSize,GuKernelSize), 1.2);
   cvtColor(imin, imin_hsv, COLOR_BGR2HSV);
   inRange(imin_hsv, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), imout);
   // Canny(imin, imout, canny1, canny2, 3);
   // dilate( imout, imout, element );
   // GaussianBlur(imout, imout, Size(7,7), 1.2);
   bitwise_not(imout, imout);
   imshow(window_capture_name, imout);
   // erode(imout, imout, element );

   findContours( imout, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_KCOS, Point(0, 0) );
   cout<<"num of contours: \t"<<contours.size()<<endl;
   if(contours.size()==0) {
     cout<<"\nIP is out\n~~~~~";
     return imout;
   }
   for( float i = 0; i< contours.size(); i++ ){
     // if(contourArea(contours[i])>400) isall2 = false;
     drawContours( drawing, contours, i, Scalar(0,0,255), 2, 8, hierarchy, 0, Point() );
     approxPolyDP(Mat(contours[i]), conto, 13, true);
     poly.push_back(conto);
     ar = arCalculate(conto, imout);
     if(conto.size()==1) arDifference.push_back({i, 1000, 1, 0});
     else arDifference.push_back({i, abs(ar-shapeAR), float(conto.size()), float(contourArea(conto)), ar});
   }
   int m = arDifference.size();
   int n = arDifference[0].size();
   // cout<<"unsorted:\n";
   // for (int i=0; i<m; i++)
   // {
   //     for (int j=0; j<n ;j++)
   //         cout << arDifference[i][j] << " ";
   //     cout << endl;
   // }
   sort(arDifference.begin(), arDifference.end(),sortcol);
   // cout<<"sorted:\n";
   // for (int i=0; i<m; i++)
   // {
   //     for (int j=0; j<n ;j++)
   //         cout << arDifference[i][j] << " ";
   //     cout << endl;
   // }
   // cout<<"1\n";
   for(int i=0; i<poly.size(); ++i){
     if(arDifference[i][2]==4 && arDifference[i][1]<0.5/* && arDifference[i][3]<220000*/) {
       // if(minFourArea>arDifference[i][3]) minFourArea = arDifference[i][3];
       goodIndex = arDifference[i][0];
       ind = i;
       cout<<"good poly info:\t"<<arDifference[i][0]<<'\t'<<arDifference[i][1]<<'\t'<<arDifference[i][2]<<'\t'<<arDifference[i][3]<<'\t'<<arDifference[i][4]<<endl;
       four_found = true;
       break;
     }
   }
   if(!four_found){
   for(int i=0;i<poly.size();++i){
     if(arDifference[i][3]>(imout.total()-10e4)) continue;
     // else if(foursAreBetter) goto lab1;
     else {
       goodIndex = arDifference[i][0];
       ind = i;
       cout<<"good poly info:\t"<<arDifference[i][0]<<'\t'<<arDifference[i][1]<<'\t'<<arDifference[i][2]<<'\t'<<arDifference[i][3]<<'\t'<<arDifference[i][4]<<endl;
       break;
     }
   }
  }

  // cout<<"size:\t"<<imout.total()<<endl;
  // cout<<"goodIndex:\t"<<goodIndex<<"\tpts num:\t"<<poly[goodIndex].size()<<"\tardif:\t"<<poly[goodIndex][1]<<endl;

   // goodIndex = arDifference[ind][0];


   // bool is_csorted = csort(poly[goodIndex], contor);

   if((poly[goodIndex].size() == 4 && arDifference[ind][1]<100)||(poly[goodIndex].size()==2 && contourArea(poly[goodIndex])<400)){
     // cout<<"arrived here\n";
     for (int k=0; k<poly[goodIndex].size(); ++k){
       // cout<<"bog\n";
       circle(drawing, poly[goodIndex][k], 10, Scalar(0,0,0), 3);
     }
     // cout<<"before\t"<<vecy<<'\t'<<vecz<<endl;
     rectangleGeometric(poly[goodIndex] ,drawing, vecy, vecz);
     flag = true;
   }
   // else {
   //   cout<<"arrived here\n";
   //   for (int k=0; k<poly[goodIndex].size(); ++k){
   //     cout<<"gob\n";
   //     circle(drawing, poly[goodIndex][k], 10, Scalar(0,0,0), 3);
   //   }
   //   imshow("bad poly", drawing);
   // }

   imshow("processed pic", drawing);

   imout = drawing.clone();
   cout<<"\nIP is out\n~~~~~\n";
   return imout;
}

void rectangleGeometric(vector<Point> rect, Mat pic, int& dx, int& dy){
  int xc=0, yc=0, xpc = (pic.cols/2)-1, ypc = (pic.rows/2)-1;
  for (int k=0; k<rect.size(); ++k){
    xc += rect[k].x/rect.size();
    yc += rect[k].y/rect.size();
  }
  xc += 1;
  yc += 1;
  dx = xc - xpc;
  dy = yc - ypc;
  // cout<<dx<<'\t'<<dy<<endl;
}

bool csort (vector<Point> cont, vector<Point2d> &res) {
  if(cont.size()==0)
  {
    cout << "empty" << endl;
    return false;
  }
  int xm=0, ym=0;
  for(int i=0; i<cont.size(); ++i){
    xm += cont[i].x;
    ym += cont[i].y;
  }
  xm /= cont.size();
  ym /= cont.size();
  vector<Point2d>  c1, c2, c3, c4;
  vector<Point2d>  t1, t2, t3, t4;


//////
// cout<<1<<endl;
// cout<<"size:"<<cont.size()<<endl;
// cout<<"coor:"<<xm<<'\t'<<ym<<endl;


  for(int i=0; i<cont.size(); ++i){
    //////
      if((cont[i].x<=xm)&&(cont[i].y>=ym)){
        c1.push_back(cont[i]);
      }
      if((cont[i].x>xm)&&(cont[i].y>ym)){
        c2.push_back(cont[i]);
      }
      if((cont[i].x>=xm)&&(cont[i].y<=ym)){
        c3.push_back(cont[i]);
      }
      if((cont[i].x<xm)&&(cont[i].y<ym)){
        c4.push_back(cont[i]);
      }
      //////
      // cout<<2<<endl;
  }

  // cout << c1.size() << " " << c2.size() << " " << c3.size() << " " << c4.size() << endl;

  if(c1.size()>1){
    // t1 = c1;
    // c1.clear();
    // c1 = csort(c1);
    return false;
  }
  if(c2.size()>1){
    // t2 = c2;
    // c2.clear();
    // c2 = csort(c2);
    return false;
  }
  if(c3.size()>1){
    // t3 = c3;
    // c3.clear();
    // c3 = csort(c3);
    return false;
  }
  if(c4.size()>1){
    // t4 = c4;
    // c4.clear();
    // c4 = csort(c4);
    return false;
  }
  //////
  // cout<<3<<endl;

  c1.insert(c1.end(), c2.begin(), c2.end());
  c3.insert(c3.end(), c4.begin(), c4.end());
  c1.insert(c1.end(), c3.begin(), c3.end());
  //////
  // cout<<4<<endl;
  res.clear();
  res = c1;
  return true;
}

float arCalculate(vector<Point> points,  Mat pic){
  if(points.size() == 1) return 10e6;
  // cout<<"num of pts:\t\t\t"<<points.size()<<endl;
  unsigned int max=0, may=0, mix=10e6, miy=10e6;
  Point pmax(0,0), pmay(0,0), pmix(10e6,0), pmiy(0,10e6);
  float aspRec, maxDist;
  // const f;
  bool isHorizontal = false;

  for(int i=0; i<points.size(); ++i){
    if(max<points[i].x) max = points[i].x;
    if(may<points[i].y) may = points[i].y;
    if(mix>points[i].x) mix = points[i].x;
    if(miy>points[i].y) miy = points[i].y;
  }
  for(int i=0; i<points.size(); ++i){
    if(max == points[i].x && pmax==Point(0,0)) pmax = points[i];
    if(may == points[i].y && pmay==Point(0,0)) pmay = points[i];
    if(mix == points[i].x && pmix==Point(10e6,0)) pmix = points[i];
    if(miy == points[i].y && pmiy==Point(0,10e6)) pmiy = points[i];
  }
  for(int i=0; i<points.size(); ++i){
    for(int j=0; j<points.size(); ++j){
      if(i==j) continue;
      if (points[i].x == points[j].x || points[i].y == points[j].y) isHorizontal = true;
      if(maxDist<euclideanDist(points[i], points[j])) maxDist = euclideanDist(points[i], points[j]);
    }
  }
  if(points.size()!=4 || isHorizontal){
    lab:
    // cout<<"I found\tmax:"<<max<<"\tmay:"<<may<<"\tmix:"<<mix<<"\tmiy:"<<miy<<endl;
    aspRec = float(max-mix)/float(may-miy);
    // cout<<"aspRec\t"<<aspRec<<endl;
  }
  else {
    // cout<<"tilt rect!"<<endl;
    if(euclideanDist(pmay, pmix)==0 || euclideanDist(pmix,pmiy)==0) goto lab;
    aspRec = euclideanDist(pmay, pmix)/euclideanDist(pmix,pmiy);
    if(aspRec<1) aspRec = 1.0/aspRec;
  }
  if(maxDist>(pic.cols-10) || maxDist>(pic.rows-10)) return 10e6;
  return aspRec;
}

float euclideanDist(Point p, Point q) {
    Point diff = p - q;
    return sqrt(diff.x*diff.x + diff.y*diff.y);
}
