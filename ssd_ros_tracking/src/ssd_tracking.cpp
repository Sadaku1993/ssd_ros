#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/opengl.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/ocl.hpp>

#include <ssd_ros_msgs/BoundingBox.h>
#include <ssd_ros_msgs/BoundingBoxArray.h>

using namespace std;
namespace enc = sensor_msgs::image_encodings;

const cv::Size MAX_DETECT_SIZE = cv::Size(100, 200);
const int MAX_MISS_FRAME = 10;
const double MIN_NEW_DETECT_INTERSECTION_RATE = 0.5;

// cv::Trackerのラッパー。
class MyTracker {
private:
	static int next_id;
	int id;
	int n_miss_frame = 0;
	cv::Rect2d rect;
	cv::Ptr<cv::Tracker> cv_tracker;
public:
	// フレーム画像と追跡対象(Rect)で初期化
	MyTracker(const cv::Mat& _frame, const cv::Rect2d& _rect) 
		: id(next_id++), rect(_rect) {
            cv_tracker = cv::TrackerMedianFlow::create();
            cv_tracker->init(_frame, _rect);
	}
	// 次フレームを入力にして、追跡対象の追跡(true)
	// MAX_MISS_FRAME以上検出が登録されていない場合は追跡失敗(false)
	bool update(const cv::Mat& _frame){
		n_miss_frame++;
		return cv_tracker->update(_frame, rect) && n_miss_frame < MAX_MISS_FRAME;
	}
	// 新しい検出(Rect)を登録。現在位置と近ければ受理してn_miss_frameをリセット(true)
	// そうでなければ(false)
	bool registerNewDetect(const cv::Rect2d& _new_detect){
		double intersection_rate = 1.0 * (_new_detect & rect).area() / (_new_detect | rect).area();
		bool is_registered = intersection_rate > MIN_NEW_DETECT_INTERSECTION_RATE;
		if (is_registered) n_miss_frame = 0;
		return is_registered;
	}
	// trackerの現在地を_imageに書き込む
	void draw(cv::Mat& _image) const{
		cv::rectangle(_image, rect, cv::Scalar(255, 0, 0), 2, 1);
		cv::putText(_image, cv::format("%03d", id), cv::Point(rect.x + 5, rect.y + 17), 
				cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 1, CV_AA);
	}
};

class Tracker{
public:
    cv::Rect2d rect;
    cv::Ptr<cv::Tracker> cv_tracker;
    int n_miss_frame;
    int id;
};
// 新しい検出(Rect)を登録。現在位置と近ければ受理してn_miss_frameをリセット(true)
// そうでなければ(false)
bool registerNewDetect(const cv::Rect2d& _new_detect, Tracker track){
    double intersection_rate = 1.0 * (_new_detect & track.rect).area() / (_new_detect | track.rect).area();
    bool is_registered = intersection_rate > MIN_NEW_DETECT_INTERSECTION_RATE;
    if (is_registered) track.n_miss_frame = 0;
    return is_registered;
}
// trackerの現在地を_imageに書き込む
void draw(cv::Mat& _image, Tracker track){
    cv::rectangle(_image, track.rect, cv::Scalar(255, 0, 0), 2, 1);
    cv::putText(_image, cv::format("%03d", track.id), cv::Point(track.rect.x + 5, track.rect.y + 17), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 1, CV_AA);
}


bool boxes_flag = false;
bool image_flag = false;

ssd_ros_msgs::BoundingBoxArrayConstPtr boxes_;
void boundingboxCallback(const ssd_ros_msgs::BoundingBoxArrayConstPtr& msg)
{
    boxes_ = msg;
    boxes_flag = true;
}

cv::Mat image_;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Process cv_ptr->image using OpenCV
  image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  // cv::imshow("sub", image_);
  // cv::waitKey(1);
  image_flag = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ssd_tracking");
    ros::NodeHandle n;
    
    // image_transport::ImageTransport it(n);
    // image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 10, imageCallback2);
    ros::Subscriber sub_image = n.subscribe("/usb_cam/image_raw", 10, imageCallback);
    ros::Rate rate(20);

    // cv::HOGDescriber detector;
    cv::HOGDescriptor detector;
    detector.setSVMDetector (cv::HOGDescriptor::getDefaultPeopleDetector());
    // std::vector<MyTracker> trackers;
    std::vector<Tracker> trackers;

    while(ros::ok())
    {
        if(image_flag){
            std::vector<cv::Rect> detections;
            detector.detectMultiScale(image_, detections);
            // trackerの更新(追跡に失敗したら削除)
            // for (auto t_it = trackers.begin(); t_it != trackers.end();){
            // 	t_it = (t_it->update(image_)) ? std::next(t_it) : trackers.erase(t_it);
            // }
            // 新しい検出があればそれを起点にtrackerを作成。(既存Trackerに重なる検出は無視)
            for(auto& d_rect : detections){
            	if (d_rect.size().area() > MAX_DETECT_SIZE.area()) continue;
                bool is_existing = true;
                for(auto& track : trackers){
                    is_existing = registerNewDetect(d_rect, track);
                }
                if(!is_existing)
                {
                    // cv::Ptr<cv::Tracker> tracker = cv::TrackerMedianFlow::create();
                }
                // if(!is_exsisting) trackers.push_back(MyTracker(image_, d_rect));
            }
            //フレームが空か一周した時break
            //if(frame.empty() || video.get(CV_CAP_PROP_POS_AVI_RATIO) == 1){
            //    break;
            //}
            // 人物追跡と人物検出の結果を表示
            // cv::Mat image = image_.clone();
            // for(auto& t : trackers) t.draw(image,fps);
            for(auto& d_rect : detections) cv::rectangle(image_, d_rect, cv::Scalar(0, 255, 0), 2, 1);
            cv::imshow("demo", image_);
            cv::waitKey(1);
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
