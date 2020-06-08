#include "obj_tracker.h"

using namespace std;
using namespace cv;

bool get_rectangle( Mat img, int low_rgb[3], int high_rgb[3],  vector< Point> & centro ) {
  
   bool there_are_contours=true;
    Mat img_th;
    vector<vector<Point> > contours;
    inRange(img, Scalar(low_rgb[2], low_rgb[1], low_rgb[0]), Scalar(high_rgb[2], high_rgb[1], high_rgb[0]), img_th);
    
  imshow( "inRange", img_th );
    waitKey(1);

    //Contours
    findContours( img_th, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    Mat drawing = Mat::zeros(img_th.size(), CV_8UC3 );

    for( int i = 0; i< contours.size(); i++ ) {
        Scalar color = Scalar( 0, 0, 255 );
        drawContours( drawing, contours, i, color, 2, 8 );
    }
    imshow( "contours", drawing );
    waitKey(1);
    
   vector<RotatedRect> minRect( contours.size() );
     for( size_t i = 0; i < contours.size(); i++ )
    {
        minRect[i] = minAreaRect( Mat(contours[i]) );  
    }
    
    size_t size_c=contours.size();
    cout<<"Contours size "<<size_c<<endl;
    if(size_c > 0){
      centro.resize(size_c);
      for( size_t i = 0; i< contours.size(); i++ ){  
        // rotated rectangle
        Point2f rect_points[4];
        //points (Point2f pts[]) returns 4 vertices of the rectangle 
        minRect[i].points( rect_points );
        
        centro[i] = minRect[i].center;
       // cout<<"c: "<< centro[i].x<< " "<<centro[i].y<<endl;
        for ( int j = 0; j < 4; j++ )
        {  // cout<<"vertices: "<< rect_points[j].x<< " "<<rect_points[j].y<<endl;
            line( drawing, rect_points[j], rect_points[(j+1)%4], Scalar( 255, 0, 0 ),1,8 );
        }      
       }
 
       imshow( "rectangle", drawing );
       waitKey(1);
       
       img.release();
       img_th.release();
       contours.clear();
       there_are_contours=true ;
   }
   else{
      there_are_contours= false;
   }
   return there_are_contours;
}








