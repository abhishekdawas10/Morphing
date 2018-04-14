#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>

using namespace cv;
using namespace std;

void DelaunayTriangulation(Rect rect, vector<Point2f> &points, vector< vector<int> > &dt){
   Subdiv2D subdiv(rect);

 for( vector<Point2f>::iterator it = points.begin(); it != points.end(); it++)
    {
     subdiv.insert(*it);	        
}
  vector<Vec6f> triangleList;
  subdiv.getTriangleList(triangleList);
  vector<Point2f> pt(3);
   vector<int> ind(3);

    for( int i = 0; i < triangleList.size(); i++ )
 {
   Vec6f t = triangleList[i];
    pt[0] = Point2f(t[0], t[1]);
    pt[1] = Point2f(t[2], t[3]);
    pt[2] = Point2f(t[4], t[5]);

if ( rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2])){
  for(int j = 0; j < 3; j++)
   for(int k = 0; k < points.size(); k++)
  if(abs(pt[j].x - points[k].x) < 1 && abs(pt[j].y - points[k].y) < 1){	
   ind[j] = k;											
}
      dt.push_back(ind);
   }
  }		
}

void morphing(Mat &img1, Mat &img2, Mat &img, vector<Point2f> &tie1, vector<Point2f> &tie2, vector<Point2f> &tiem, double alpha)
{
    
    Rect br = boundingRect(tiem);          // These are the bounding rectangles of three images. This is done to maintain efficiency. 
                                          // we need to warp only the triangle part at a time not the whole image.
    Rect br1 = boundingRect(tie1);
    Rect br2 = boundingRect(tie2);
    
    
    vector<Point2f> t1Cropped;
     vector<Point2f> t2Cropped;
      vector<Point2f> tCropped; 
        vector<Point> tCroppedInt;
    for(int i = 0; i < 3; i++)
    {
        tCropped.push_back( Point2f( tiem[i].x-br.x, tiem[i].y-br.y) );// The top left corner is subtracted to get the cropped image 
                                                                            // on which I applied affine transformation.
        t1Cropped.push_back( Point2f( tie1[i].x-br1.x, tie1[i].y-br1.y) );
        t2Cropped.push_back( Point2f( tie2[i].x-br2.x, tie2[i].y-br2.y) );
          tCroppedInt.push_back( Point(tiem[i].x-br.x, tiem[i].y-br.y) ); 
        
    }
    
   
    Mat mask = Mat::zeros(br.height, br.width, CV_32FC3);
    fillConvexPoly(mask,tCroppedInt,Scalar(1.0, 1.0, 1.0), 16, 0); // The region in rect outside the triangle is of no interest.
    
    Mat img1Cropped, img2Cropped;
    img1(br1).copyTo(img1Cropped);
    img2(br2).copyTo(img2Cropped);
//    cout<<"img1"<<img1Cropped.at<float>(0,1)<<endl;
    Mat warpImage1 (br.height, br.width,CV_8UC1, Scalar(0));
     Mat warpImage2 (br.height, br.width,CV_8UC1, Scalar(0));
    Mat warpMat1(2, 3, CV_32FC1);
      Mat warpMat2(2, 3, CV_32FC1);
    warpMat1 = getAffineTransform(t1Cropped,tCropped); // Tranformation matrix is found for first img
   // cout<<"warp="<<warpMat1.rows<<" "<<warpMat1.cols<<" "<<warpMat1.at<float>(0,1);
     
    for(int a=0; a<warpImage1.rows; a++){
       for (int b=0; b<warpImage1.cols; b++){
       int xd = (warpMat1.at<double>(0,0))*a + (warpMat1.at<double>(0,1))*b + (warpMat1.at<double>(0,2));
       int yd = (warpMat1.at<double>(1,0))*a + (warpMat1.at<double>(1,1))*b + (warpMat1.at<double>(1,2));
         if(xd<img1Cropped.rows && xd>=0 && yd<img1Cropped.cols && yd>=0){
       //  warpImage1.at<uchar>(a,b) = (img1Cropped.at<uchar>(xd,yd));
       }
       
       }
     
     }
     
     warpAffine( img1Cropped, warpImage1, warpMat1, warpImage1.size(),INTER_LINEAR,BORDER_REFLECT_101); // Transformation is applied.
    
     warpMat2 = getAffineTransform(t2Cropped,tCropped); // Tranformation matrix is found for second img
     
   /*  
       for(int a=0; a<warpImage2.rows; a++){
       for (int b=0; b<warpImage2.cols; b++){
       int xd = (warpMat2.at<float>(0,0))*a + (warpMat2.at<float>(0,1))*b + (warpMat2.at<float>(0,2));
       int yd = (warpMat2.at<float>(1,0))*a + (warpMat2.at<float>(1,1))*b + (warpMat2.at<float>(1,2));
       warpImage2.at<float>(a,b) = (img2Cropped.at<float>(xd,yd));
       
       }
     
     }*/
     
    warpAffine( img2Cropped, warpImage2, warpMat2, warpImage2.size(), INTER_LINEAR,BORDER_REFLECT_101); // Transformation is applied.
  //  
   
    Mat imgCropped = ((1 - alpha)*warpImage1)+(alpha*warpImage2);
    
    multiply(imgCropped,mask,imgCropped);                    // The triangular part from rect is copied to the output image.  
    multiply(img(br), Scalar(1.0,1.0,1.0) - mask, img(br));
    img(br) = img(br) + imgCropped;
    
    
}

int main( int argc, char** argv)
{
    
    double alpha;
//cout<<"Please enter the value of alpha between 0 to 1\n";
   // cin>>alpha;
   // for(double alpha=0;alpha<=1;alpha++){
   
    cout<<"1. Morphing between an image and its affine transformation\n2. Morphing between two images A and B\n";
    int t;
    cin>>t;
    string image1,image2,txt1,txt2;
    if(t==1){
    image1="rotation_inbuilt.jpg";
    image2="sample5.jpg";
    txt1="sample5.txt";
    txt2="sample5.txt";
    } 
    if(t==2){
    image1="ted_cruz.jpg";
    image2="trumph.jpg";
    txt1="ted_cruz.jpg.txt";
    txt2="trumph.jpg.txt";
    }
    
    for(alpha=0;alpha<=1;alpha=alpha+0.1){
    //Read input images
    Mat imgg1 = imread(image1);
    Mat imgg2 = imread(image2);
    int r1=imgg1.rows;
    int r2=imgg2.rows;
    int c1=imgg1.cols;
    int c2=imgg2.cols;
     
   Mat img1(r1,c1,CV_32F,Scalar(0));
    Mat img2(r2,c2,CV_32F,Scalar(0));
    imgg1.convertTo(img1, CV_32F);
    imgg2.convertTo(img2, CV_32F);
    
    Mat morphedImg = Mat::zeros(img1.size(), CV_32FC3);
   
      vector<Point2f> points1,points2;
    if(t==1){
    ifstream ifs("sample5.txt");
    int x1, y1;
    int xm,ym;
    while(ifs >> x1 >> y1)
    {
        points1.push_back(Point2f(x1,y1));
        
    }
    
  
    ifstream ifs2("sample5.txt");
  
    while(ifs2 >> x1 >> y1)
    {
        points2.push_back(Point2f(x1,y1));
    }
   }
   
   if(t==2){
   
    ifstream ifs("ted_cruz.jpg.txt");
    int x1, y1;
    int xm,ym;
    while(ifs >> x1 >> y1)
    {
        points1.push_back(Point2f(x1,y1));
        
    }
    
  
    ifstream ifs2("trumph.txt");
  
    while(ifs2 >> x1 >> y1)
    {
        points2.push_back(Point2f(x1,y1));
    }
   
   }
    vector<Point2f> points;
    
    //Points on intermediate image is calculated
    for(int i = 0; i < points1.size(); i++)
    {
        float x, y;
        x = (1 - alpha) * points1[i].x + alpha * points2[i].x;
        y =  ( 1 - alpha ) * points1[i].y + alpha * points2[i].y;
        
        points.push_back(Point2f(x,y));
        
    }
    int i=0;
     vector< vector<int> > dt;
	Rect rect(0, 0, img2.cols, img2.rows);
DelaunayTriangulation(rect, points, dt);
ofstream ofs("triangle.txt");
  for(int l=0;l<dt.size();l++)
  {
  for(int g=0;g<3;g++){

    ofs<<dt[l][g]<<" ";
  
  }
  ofs<<endl;

}
//cout<<dt[0][0];
    
   //Read triangle indices
    ifstream ifs3("triangle.txt");
    int x,y,z;
    
    while(ifs3 >> x >> y >> z)
    {
       
        vector<Point2f> tie1, tie2, tiem;
   tie1.push_back(points1[x]);
    tie1.push_back(points1[y]);          // corresponding points in img1 are found.
     tie1.push_back(points1[z]);
        
    
    tie2.push_back(points2[x]);    
    tie2.push_back(points2[y]);        // corresponding points in img2 are found.
    tie2.push_back(points2[z]);
        
  
     tiem.push_back(points[x]);
     tiem.push_back(points[y]);
     tiem.push_back(points[z]);
        
        morphing(img1, img2, morphedImg, tie1, tie2, tiem, alpha);
        
    }
    imshow("MorphedFace.jpg", morphedImg/255.0 );
    
    if(alpha==0){
    imwrite("MorphedFace0.jpg", morphedImg/255.0 );}
     if(alpha==0.1){
    imwrite("MorphedFace0.1.jpg", morphedImg/255.0 );}
     if(alpha==0.2){
    imwrite("MorphedFace0.2.jpg", morphedImg/255.0 );}
     if(alpha==0.3){
    imwrite("MorphedFace0.3.jpg", morphedImg/255.0 );}
     if(alpha==0.4){
    imwrite("MorphedFace0.4.jpg", morphedImg/255.0 );}
     if(alpha==0.5){
    imwrite("MorphedFace0.5.jpg", morphedImg/255.0 );}
     if(alpha==0.6){
    imwrite("MorphedFace0.6.jpg", morphedImg/255.0 );}
     if(alpha==0.7){
    imwrite("MorphedFace0.7.jpg", morphedImg/255.0 );}
     if(alpha==0.8){
    imwrite("MorphedFace0.8.jpg", morphedImg/255.0 );}
    
    
    waitKey(500);
    }
  
    return 0;
}


