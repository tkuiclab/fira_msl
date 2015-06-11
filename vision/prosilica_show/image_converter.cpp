#include "image_converter.hpp"

#define REDITEM 0x01
#define GREENITEM 0x02
#define BLUEITEM 0x04
#define YELLOWITEM 0x08
#define FILE_PATH "/tmp/HSVcolormap.bin"
#define IMAGE_TEST1 "src/vision/resources/1.bmp"
#define IMAGE_TEST2 "src/vision/resources/2.bmp"
#define IMAGE_TEST3 "src/vision/resources/3.bmp"
#define IMAGE_TEST4 "src/vision/resources/4.bmp"

using namespace std;
using namespace cv;
typedef unsigned char BYTE;

ImageConverter::ImageConverter()
   :it_(nh)
{
    image_pub_ = it_.advertise("/Image_pkg/Show_image", 1);
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);

    color_map = ColorFile();
    get_scan();
} 

ImageConverter::~ImageConverter()
{

}


void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_ptr->image = imread( IMAGE_TEST4 , CV_LOAD_IMAGE_COLOR );
    Mat Redmap(Size(cv_ptr->image.cols,cv_ptr->image.rows),CV_8UC3);
    Mat Greenmap(Size(cv_ptr->image.cols,cv_ptr->image.rows),CV_8UC3);
    Mat Bluemap(Size(cv_ptr->image.cols,cv_ptr->image.rows),CV_8UC3);
    Mat Yellowmap(Size(cv_ptr->image.cols,cv_ptr->image.rows),CV_8UC3);
    HSVmap(cv_ptr->image,Redmap,Greenmap,Bluemap,Yellowmap);
    int Redmap_x_max,Redmap_x_min,Redmap_y_max,Redmap_y_min;
    int Greenmap_x_max,Greenmap_x_min,Greenmap_y_max,Greenmap_y_min;
    int Bluemap_x_max,Bluemap_x_min,Bluemap_y_max,Bluemap_y_min;
    int Yellowmap_x_max,Yellowmap_x_min,Yellowmap_y_max,Yellowmap_y_min;

    objectdet(Redmap,Redmap_x_max,Redmap_x_min,Redmap_y_max,Redmap_y_min);
    objectdet(Bluemap,Bluemap_x_max,Bluemap_x_min,Bluemap_y_max,Bluemap_y_min);
    objectdet(Yellowmap,Yellowmap_x_max,Yellowmap_x_min,Yellowmap_y_max,Yellowmap_y_min);
    place_case(Greenmap,Greenmap_x_max,Greenmap_x_min,Greenmap_y_max,Greenmap_y_min);

    if((Redmap_x_max!=0) && (Redmap_x_min!=0) && (Redmap_y_max!=0) && (Redmap_y_min!=0)){
        draw(cv_ptr->image,Redmap_x_max,Redmap_x_min,Redmap_y_max,Redmap_y_min);
    }
    if((Greenmap_x_max!=0) && (Greenmap_x_min!=0) && (Greenmap_y_max!=0) && (Greenmap_y_min!=0)){
        draw(cv_ptr->image,Greenmap_x_max,Greenmap_x_min,Greenmap_y_max,Greenmap_y_min);
    }
    if((Bluemap_x_max!=0) && (Bluemap_x_min!=0) && (Bluemap_y_max!=0) && (Bluemap_y_min!=0)){
        draw(cv_ptr->image,Bluemap_x_max,Bluemap_x_min,Bluemap_y_max,Bluemap_y_min);
    }
    if((Yellowmap_x_max!=0) && (Yellowmap_x_min!=0) && (Yellowmap_y_max!=0) && (Yellowmap_y_min!=0)){
        draw(cv_ptr->image,Yellowmap_x_max,Yellowmap_x_min,Yellowmap_y_max,Yellowmap_y_min);
    }


//    cv::imshow("Redmap", Redmap);
//    cv::imshow("Greenmap", Greenmap);
//    cv::imshow("Bluemap", Bluemap);
//    cv::imshow("Yellowmap", Yellowmap);
   cv::imshow("Image", cv_ptr->image);
   cv::waitKey(10);
    
}
vector<BYTE> ImageConverter::ColorFile()
{
    // open the file:
    streampos fileSize;
    std::ifstream file(FILE_PATH, ios::binary);
    // get its size:
    file.seekg(0, ios::end);
    fileSize = file.tellg();
    file.seekg(0, ios::beg);
    // read the data:
    vector<BYTE> fileData(fileSize);
    file.read((char*) &fileData[0], fileSize);
    return fileData;
}
void ImageConverter::HSVmap(Mat frame, Mat Redmap, Mat Greenmap, Mat Bluemap, Mat Yellowmap){
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            unsigned char B = frame.data[(i*frame.cols*3)+(j*3)+0];
            unsigned char G = frame.data[(i*frame.cols*3)+(j*3)+1];
            unsigned char R = frame.data[(i*frame.cols*3)+(j*3)+2];
            if(color_map[R+(G<<8)+(B<<16)] & REDITEM){
                Redmap.data[(i*Redmap.cols*3)+(j*3)+0] = 255;
                Redmap.data[(i*Redmap.cols*3)+(j*3)+1] = 255;
                Redmap.data[(i*Redmap.cols*3)+(j*3)+2] = 255;
            }else{
                Redmap.data[(i*Redmap.cols*3)+(j*3)+0] = 0;
                Redmap.data[(i*Redmap.cols*3)+(j*3)+1] = 0;
                Redmap.data[(i*Redmap.cols*3)+(j*3)+2] = 0;
            }
            if(color_map[R+(G<<8)+(B<<16)] & GREENITEM){
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+0] = 255;
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+1] = 255;
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+2] = 255;
            }else{
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+0] = 0;
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+1] = 0;
                Greenmap.data[(i*Greenmap.cols*3)+(j*3)+2] = 0;
            }
            if(color_map[R+(G<<8)+(B<<16)] & BLUEITEM){
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+0] = 255;
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+1] = 255;
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+2] = 255;
            }else{
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+0] = 0;
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+1] = 0;
                Bluemap.data[(i*Bluemap.cols*3)+(j*3)+2] = 0;
            }
            if(color_map[R+(G<<8)+(B<<16)] & YELLOWITEM){
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+0] = 255;
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+1] = 255;
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+2] = 255;
            }else{
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+0] = 0;
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+1] = 0;
                Yellowmap.data[(i*Yellowmap.cols*3)+(j*3)+2] = 0;
            }
        }
    }
}
void ImageConverter::get_scan(){
    nh.getParam("/FIRA/Scan/Parameter",scan_para);
    nh.getParam("/FIRA/Scan/Near",scan_near);
    nh.getParam("/FIRA/Scan/Middle",scan_middle);
    nh.getParam("/FIRA/Scan/Far",scan_far);
}
void ImageConverter::objectdet(Mat frame, int &x_max, int &x_min, int &y_max, int &y_min){
    int neardis   = (scan_para[3]-scan_para[0])/scan_para[2]*2;
    int middledis = (scan_para[5]-scan_para[3])/scan_para[4]*2;
    int fardis    = (scan_para[7]-scan_para[5])/scan_para[6]*2;
    int nearangle   = 360/(scan_para[1]/10);
    int middleangle = nearangle*2;
    int farangle    = middleangle*2;
    deque<int>nearpoint;
    deque<int>middlepoint;
    deque<int>farpoint;
    int nearnum_max[2] = {0};//1.num_adress 2.point_num
    int middlenum_max[2] = {0};
    int farnum_max[2] = {0};
    int objectnum_max[3] = {0};//1.nmf_adress 2.num_adress 3.point_num
    objectdet_search(frame, scan_near,nearpoint, nearangle, neardis,1);
    objectdet_search(frame, scan_middle,middlepoint, middleangle, middledis,2);
    objectdet_search(frame, scan_far,farpoint, farangle, fardis,3);
    if((!near_num.empty()) || (!middle_num.empty()) || (!far_num.empty())){
        if(!near_num.empty()){
            nearnum_max[0]=0;nearnum_max[1]=near_num[0];
            for(int i=1;i<near_num.size();i++){
                if(near_num[i]>nearnum_max[1]){
                    nearnum_max[0]=i;nearnum_max[1]=near_num[i];
                }
            }
        }
        if(!middle_num.empty()){
            middlenum_max[0]=0;middlenum_max[1]=middle_num[0];
            for(int i=1;i<middle_num.size();i++){
                if(middle_num[i]>middlenum_max[1]){
                    middlenum_max[0]=i;middlenum_max[1]=middle_num[i];
                }
            }
        }
        if(!far_num.empty()){
            farnum_max[0]=0;farnum_max[1]=far_num[0];
            for(int i=1;i<far_num.size();i++){
                if(far_num[i]>farnum_max[1]){
                    farnum_max[0]=i;farnum_max[1]=far_num[i];
                }
            }
        }
        if(nearnum_max[1] > middlenum_max[1]){
            if(nearnum_max[1] > farnum_max[1]){
                objectnum_max[0] = 1;
                objectnum_max[1] = nearnum_max[0];
                objectnum_max[2] = nearnum_max[1];
            }else{
                objectnum_max[0] = 3;
                objectnum_max[1] = farnum_max[0];
                objectnum_max[2] = farnum_max[1];
            }
        }else{
            if(middlenum_max[1] > farnum_max[1]){
                objectnum_max[0] = 2;
                objectnum_max[1] = middlenum_max[0];
                objectnum_max[2] = middlenum_max[1];
            }else{
                objectnum_max[0] = 3;
                objectnum_max[1] = farnum_max[0];
                objectnum_max[2] = farnum_max[1];
            }
        }
        int point_bef = 0;
        if(objectnum_max[0]==1){
            for(int i=0;i<objectnum_max[1];i++)
                point_bef += near_num[i]*2;
            x_max = near_point[1+point_bef];
            x_min = near_point[1+point_bef];
            y_max = near_point[point_bef];
            y_min = near_point[point_bef];
            for(int i=0;i<objectnum_max[2]*2;i+=2){
                int x = near_point[i+1+point_bef];
                int y = near_point[i+point_bef];
                if(x_max<x)x_max=near_point[i+1+point_bef];
                if(x_min>x)x_min=near_point[i+1+point_bef];
                if(y_max<y)y_max=near_point[i+point_bef];
                if(y_min>y)y_min=near_point[i+point_bef];
            }
            //draw(frame, x_max, x_min, y_max, y_min);

        }else if(objectnum_max[0]==2){
            for(int i=0;i<objectnum_max[1];i++)
                point_bef += middle_num[i]*2;
            x_max = middle_point[1+point_bef];
            x_min = middle_point[1+point_bef];
            y_max = middle_point[point_bef];
            y_min = middle_point[point_bef];
            for(int i=0;i<objectnum_max[2]*2;i+=2){
                int x = middle_point[i+1+point_bef];
                int y = middle_point[i+point_bef];
                if(x_max<x)x_max=middle_point[i+1+point_bef];
                if(x_min>x)x_min=middle_point[i+1+point_bef];
                if(y_max<y)y_max=middle_point[i+point_bef];
                if(y_min>y)y_min=middle_point[i+point_bef];
            }
            //draw(frame, x_max, x_min, y_max, y_min);
        }else if(objectnum_max[0]==3){
            for(int i=0;i<objectnum_max[1];i++)
                point_bef += far_num[i]*2;
            x_max = far_point[1+point_bef];
            x_min = far_point[1+point_bef];
            y_max = far_point[point_bef];
            y_min = far_point[point_bef];
            for(int i=0;i<objectnum_max[2]*2;i+=2){
                int x = far_point[i+1+point_bef];
                int y = far_point[i+point_bef];
                if(x_max<x)x_max=far_point[i+1+point_bef];
                if(x_min>x)x_min=far_point[i+1+point_bef];
                if(y_max<y)y_max=far_point[i+point_bef];
                if(y_min>y)y_min=far_point[i+point_bef];
            }
            //draw(frame, x_max, x_min, y_max, y_min);
        }
    }else{
        x_max = 0;x_min = 0;y_max = 0;y_min = 0;
    }
    near_num.clear();near_point.clear();
    middle_num.clear();middle_point.clear();
    far_num.clear();far_point.clear();
}
void ImageConverter::objectdet_search(Mat frame, vector<int> &scan, deque<int> &point, int angle, int distance, int level ){

    int neardis   = (scan_para[3]-scan_para[0])/scan_para[2]*2;
    int middledis = (scan_para[5]-scan_para[3])/scan_para[4]*2;
    int fardis    = (scan_para[7]-scan_para[5])/scan_para[6]*2;
    int nearangle   = 360/(scan_para[1]/10);
    int middleangle = nearangle*2;
    int farangle    = middleangle*2;

    int dis,ang;
    if(!scan.empty()){
        for(int i=0;i<angle;i++){
            for(int j=0;j<distance;j=j+2){
                num = 0;
                if(frame.data[(scan[(i*distance)+j+1]*frame.cols*3)+(scan[(i*distance)+j]*3)+0] == 255){
                    objectdet_point(frame,scan,point,distance,i,j,level);
                    num = 1;
                    while(!point.empty()){
                        dis = point.front(); point.pop_front();
                        ang = point.front(); point.pop_front();
                        objectdet_arund(frame,scan,point,angle,distance,ang,dis,level);
                        if(dis==(distance-2)){
                            if(level==1){
                                near_last.push_back(ang);
                                near_last.push_back(dis);
                            }else if(level==2){
                                middle_last.push_back(ang);
                                middle_last.push_back(dis);
                            }
                        }
                    }

                    if((level==1)&&(!near_last.empty())){
                        deque<int> neartomid_point;
                        for(int po=0;po<near_last.size();po+=2){
                            int neartomid_ang =near_last[po]*2;
                            if(frame.data[(scan_middle[(neartomid_ang*middledis)+0+1]*frame.cols*3)+(scan_middle[(neartomid_ang*middledis)+0]*3)+0]==255){
                                neartomid_point.push_back(0);
                                neartomid_point.push_back(neartomid_ang);
                            }
                        }
                        while (!neartomid_point.empty()) {
                            int ntom_dis = neartomid_point.front(); neartomid_point.pop_front();
                            int ntom_ang = neartomid_point.front(); neartomid_point.pop_front();
                            objectdet_arund(frame,scan_middle,neartomid_point,middleangle,middledis,ntom_ang,ntom_dis,1);
                            if(ntom_dis==(middledis-2)){
                                middle_last.push_back(ntom_ang);
                                middle_last.push_back(ntom_dis);
                            }
                        }
                        near_last.clear();

                        if(!middle_last.empty()){
                            deque<int> midtofar_point;
                            for(int po=0;po<middle_last.size();po+=2){
                                int midtofar_ang =middle_last[po]*2;
                                if(frame.data[(scan_far[(midtofar_ang*fardis)+0+1]*frame.cols*3)+(scan_far[(midtofar_ang*fardis)+0]*3)+0]==255){
                                    midtofar_point.push_back(0);
                                    midtofar_point.push_back(midtofar_ang);
                                }
                            }
                            while (!midtofar_point.empty()) {
                                int mtof_dis = midtofar_point.front(); midtofar_point.pop_front();
                                int mtof_ang = midtofar_point.front(); midtofar_point.pop_front();
                                objectdet_arund(frame,scan_far,midtofar_point,farangle,fardis,mtof_ang,mtof_dis,1);
                            }
                            middle_last.clear();
                        }

                    }
                    if((level==2)&&(!middle_last.empty())){
                        deque<int> midtofar_point;
                        for(int po=0;po<middle_last.size();po+=2){
                            int midtofar_ang =middle_last[po]*2;
                            if(frame.data[(scan_far[(midtofar_ang*fardis)+0+1]*frame.cols*3)+(scan_far[(midtofar_ang*fardis)+0]*3)+0]==255){
                                midtofar_point.push_back(0);
                                midtofar_point.push_back(midtofar_ang);
                            }
                        }
                        while (!midtofar_point.empty()) {
                            int mtof_dis = midtofar_point.front(); midtofar_point.pop_front();
                            int mtof_ang = midtofar_point.front(); midtofar_point.pop_front();
                            objectdet_arund(frame,scan_far,midtofar_point,farangle,fardis,mtof_ang,mtof_dis,2);
                        }
                        middle_last.clear();
                    }
                    if(level==1)near_num.push_back(num);
                    else if(level==2)middle_num.push_back(num);
                    else if(level==3)far_num.push_back(num);
                }
            }
        }
    }
}
void ImageConverter::objectdet_point(Mat frame, vector<int> &scan, deque<int> &point,int distance, int ang, int dis, int level ){
    frame.data[(scan[(ang*distance)+dis+1]*frame.cols*3)+(scan[(ang*distance)+dis]*3)+0] =0;
    frame.data[(scan[(ang*distance)+dis+1]*frame.cols*3)+(scan[(ang*distance)+dis]*3)+1] =0;
    frame.data[(scan[(ang*distance)+dis+1]*frame.cols*3)+(scan[(ang*distance)+dis]*3)+2] =0;
    point.push_back(dis);point.push_back(ang);
    if(level==1){
        near_point.push_back(scan[(ang*distance)+dis]);
        near_point.push_back(scan[(ang*distance)+dis+1]);
    }else if(level==2){
        middle_point.push_back(scan[(ang*distance)+dis]);
        middle_point.push_back(scan[(ang*distance)+dis+1]);
    }else if(level==3){
        far_point.push_back(scan[(ang*distance)+dis]);
        far_point.push_back(scan[(ang*distance)+dis+1]);
    }
    num += 1;
}
void ImageConverter::objectdet_arund(Mat frame, vector<int> &scan, deque<int> &point, int angle,int distance, int ang, int dis, int level ){
    int dis_new,ang_new;
    if(dis!=0){
        dis_new = dis-2;
        if(ang==0) ang_new = angle-1;
        else ang_new = ang-1;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang-1 dis-2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);
    }

    dis_new = dis;
    if(ang==0) ang_new = angle-1;
    else ang_new = ang-1;
    if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang-1 dis+0
        objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);

    if( dis!=(distance-2)){
        dis_new = dis+2;
        if(ang==0) ang_new = angle-1;
        else ang_new = ang-1;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang-1 dis+2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);
    }

    if(dis!=0){
        dis_new = dis-2;
        if( ang==(angle-1)) ang_new=0;
        else ang_new = ang+1;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang+1 dis-2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);
    }

    dis_new = dis;
    if( ang==(angle-1)) ang_new=0;
    else ang_new = ang+1;
    if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang+1 dis+0
        objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);

    if( dis!=(distance-2)){
        dis_new = dis+2;
        if( ang==(angle-1)) ang_new=0;
        else ang_new = ang+1;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang+1 dis+2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);
    }

    if( dis!=(distance-2)){
        dis_new = dis+2;ang_new = ang;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang+0 dis+2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);
    }

    if(dis!=0){
        dis_new = dis-2;ang_new = ang;
        if(frame.data[(scan[(ang_new*distance)+dis_new+1]*frame.cols*3)+(scan[(ang_new*distance)+dis_new]*3)+0] ==255)//ang+0 dis-2
            objectdet_point(frame,scan,point,distance,ang_new,dis_new,level);

    }
}
void ImageConverter::draw(Mat frame, int x_max, int x_min, int y_max, int y_min){
    for(int i=x_min;i<=x_max;i++){
        frame.data[(i*frame.cols*3)+(y_min*3)+0] = 0;
        frame.data[(i*frame.cols*3)+(y_min*3)+1] = 255;
        frame.data[(i*frame.cols*3)+(y_min*3)+2] = 0;
        frame.data[(i*frame.cols*3)+(y_max*3)+0] = 0;
        frame.data[(i*frame.cols*3)+(y_max*3)+1] = 255;
        frame.data[(i*frame.cols*3)+(y_max*3)+2] = 0;
    }
    for(int i=y_min;i<=y_max;i++){
        frame.data[(x_min*frame.cols*3)+(i*3)+0] = 0;
        frame.data[(x_min*frame.cols*3)+(i*3)+1] = 255;
        frame.data[(x_min*frame.cols*3)+(i*3)+2] = 0;
        frame.data[(x_max*frame.cols*3)+(i*3)+0] = 0;
        frame.data[(x_max*frame.cols*3)+(i*3)+1] = 255;
        frame.data[(x_max*frame.cols*3)+(i*3)+2] = 0;
    }
}
void ImageConverter::place_case(Mat frame, int &x_max, int &x_min, int &y_max, int &y_min){
    vector<int> place_point;
    for(int i=0;i<scan_near.size();i+=2){
        if(frame.data[(scan_near[i+1]*frame.cols*3)+(scan_near[i]*3)+0] == 255){
            place_point.push_back(scan_near[i]);
            place_point.push_back(scan_near[i+1]);
        }
    }
    for(int i=0;i<scan_middle.size();i+=2){
        if(frame.data[(scan_middle[i+1]*frame.cols*3)+(scan_middle[i]*3)+0] == 255){
            place_point.push_back(scan_middle[i]);
            place_point.push_back(scan_middle[i+1]);
        }
    }
    for(int i=0;i<scan_far.size();i+=2){
        if(frame.data[(scan_far[i+1]*frame.cols*3)+(scan_far[i]*3)+0] == 255){
            place_point.push_back(scan_far[i]);
            place_point.push_back(scan_far[i+1]);
        }
    }
    x_max = place_point[1]; x_min = place_point[1];
    y_max = place_point[0]; y_min = place_point[0];
    for(int i=0;i<place_point.size();i+=2){
        if(x_max<place_point[i+1])x_max=place_point[i+1];
        if(x_min>place_point[i+1])x_min=place_point[i+1];
        if(y_max<place_point[i])y_max=place_point[i];
        if(y_min>place_point[i])y_min=place_point[i];
    }
}
void ImageConverter::objectdet_color(Mat frame, Mat Redmap, Mat Greenmap, Mat Bluemap, Mat Yellowmap )
{
    for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
            if(Redmap.data[(i*frame.cols*3)+(j*3)+0] != 0){
				frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 255;
			}
		}
	}
	for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
			if(Greenmap.data[(i*frame.cols*3)+(j*3)+0] != 0){
				frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
			}
		}
	}
	for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
			if(Bluemap.data[(i*frame.cols*3)+(j*3)+0] != 0){
				frame.data[(i*frame.cols*3)+(j*3)+0] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 0;
			}
		}
	}
	for(int i=0;i<frame.rows;i++){
        for(int j=0;j<frame.cols;j++){
			if(Yellowmap.data[(i*frame.cols*3)+(j*3)+0] != 0){
				frame.data[(i*frame.cols*3)+(j*3)+0] = 0;
                frame.data[(i*frame.cols*3)+(j*3)+1] = 255;
                frame.data[(i*frame.cols*3)+(j*3)+2] = 255;
			}
		}
	}
}

