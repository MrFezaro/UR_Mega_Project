#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <filesystem>
#include <map>
#include <optional>
#include <sstream>

struct ColorRange {
    std::string name;
    cv::Scalar  lower1, upper1;
    cv::Scalar  lower2, upper2;
    cv::Scalar  drawColor;
    bool        dual = false;
};

struct TuneState {
    int h_low  = 0,   h_high = 180;
    int s_low  = 100, s_high = 255;
    int v_low  = 100, v_high = 255;
};

static void on_h_low  (int v, void* d){ ((TuneState*)d)->h_low  = v; }
static void on_h_high (int v, void* d){ ((TuneState*)d)->h_high = v; }
static void on_s_low  (int v, void* d){ ((TuneState*)d)->s_low  = v; }
static void on_s_high (int v, void* d){ ((TuneState*)d)->s_high = v; }
static void on_v_low  (int v, void* d){ ((TuneState*)d)->v_low  = v; }
static void on_v_high (int v, void* d){ ((TuneState*)d)->v_high = v; }

class ColorSquareDetector : public rclcpp::Node
{
public:
    ColorSquareDetector() : Node("color_square_detector")
    {
        this->declare_parameter("min_area",       1500);
        this->declare_parameter("approx_eps",     0.04);
        this->declare_parameter("side_ratio_min", 0.60);
        this->declare_parameter("aspect_min",     0.50);
        this->declare_parameter("aspect_max",     2.00);
        this->declare_parameter("tune_mode",      false);
        this->declare_parameter("tune_color",     std::string("Red"));
        this->declare_parameter("params_file",    std::string("color_params.yaml"));

        params_file_    = this->get_parameter("params_file").as_string();
        min_area_       = this->get_parameter("min_area").as_int();
        approx_eps_     = this->get_parameter("approx_eps").as_double();
        side_ratio_min_ = this->get_parameter("side_ratio_min").as_double();
        aspect_min_     = this->get_parameter("aspect_min").as_double();
        aspect_max_     = this->get_parameter("aspect_max").as_double();
        tune_mode_      = this->get_parameter("tune_mode").as_bool();
        tune_color_     = this->get_parameter("tune_color").as_string();

        colors_ = {
            {
                "Red",
                cv::Scalar(0,   100, 100), cv::Scalar(10,  255, 255),
                cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255),
                cv::Scalar(0, 0, 255), true
            },
            {
                "Green",
                cv::Scalar(40, 50, 50), cv::Scalar(90, 255, 255),
                {}, {}, cv::Scalar(0, 200, 0), false
            },
            {
                "Blue",
                cv::Scalar(100, 100, 50), cv::Scalar(130, 255, 255),
                {}, {}, cv::Scalar(255, 100, 0), false
            }
        };

        loadParams();

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&ColorSquareDetector::imageCallback, this, std::placeholders::_1));

        img_pub_  = this->create_publisher<sensor_msgs::msg::Image>("/detected/image", 10);
        det_pub_  = this->create_publisher<std_msgs::msg::String>("/cube_detections", 10);

        if (tune_mode_) {
            for (size_t i = 0; i < colors_.size(); i++) {
                if (colors_[i].name == tune_color_) { tune_idx_ = (int)i; break; }
            }
            if (tune_idx_ < 0) {
                RCLCPP_WARN(this->get_logger(), "tune_color '%s' not found, defaulting to Red", tune_color_.c_str());
                tune_idx_ = 0;
            }
            auto & c     = colors_[tune_idx_];
            tune_.h_low  = (int)c.lower1[0]; tune_.h_high = (int)c.upper1[0];
            tune_.s_low  = (int)c.lower1[1]; tune_.s_high = (int)c.upper1[1];
            tune_.v_low  = (int)c.lower1[2]; tune_.v_high = (int)c.upper1[2];

            tune_win_ = "HSV Tuner " + tune_color_;
            cv::namedWindow(tune_win_, cv::WINDOW_NORMAL);
            cv::resizeWindow(tune_win_, 1280, 540);
            cv::createTrackbar("H low",  tune_win_, nullptr, 180, on_h_low,  &tune_);
            cv::createTrackbar("H high", tune_win_, nullptr, 180, on_h_high, &tune_);
            cv::createTrackbar("S low",  tune_win_, nullptr, 255, on_s_low,  &tune_);
            cv::createTrackbar("S high", tune_win_, nullptr, 255, on_s_high, &tune_);
            cv::createTrackbar("V low",  tune_win_, nullptr, 255, on_v_low,  &tune_);
            cv::createTrackbar("V high", tune_win_, nullptr, 255, on_v_high, &tune_);
            cv::setTrackbarPos("H low",  tune_win_, tune_.h_low);
            cv::setTrackbarPos("H high", tune_win_, tune_.h_high);
            cv::setTrackbarPos("S low",  tune_win_, tune_.s_low);
            cv::setTrackbarPos("S high", tune_win_, tune_.s_high);
            cv::setTrackbarPos("V low",  tune_win_, tune_.v_low);
            cv::setTrackbarPos("V high", tune_win_, tune_.v_high);
        }

        RCLCPP_INFO(this->get_logger(), "Color Square Detector started. Publishing detections to /cube_detections");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        int img_w = frame.cols;
        int img_h = frame.rows;

        cv::Mat blurred;
        cv::GaussianBlur(frame, blurred, cv::Size(7, 7), 0);
        cv::Mat hsv;
        cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

        if (tune_mode_ && tune_idx_ >= 0) {
            auto & c = colors_[tune_idx_];
            c.lower1 = cv::Scalar(tune_.h_low,  tune_.s_low,  tune_.v_low);
            c.upper1 = cv::Scalar(tune_.h_high, tune_.s_high, tune_.v_high);
        }

        // Detect each color and track the best (largest) centroid per color
        std::map<std::string, cv::Point2f> detections;
        for (auto & color : colors_) {
            cv::Mat mask = buildMask(hsv, color);
            mask = cleanMask(mask);
            auto centroid = detectBest(frame, mask, color);
            if (centroid.has_value()) {
                detections[color.name] = centroid.value();
            }
        }

        // Publish JSON: {"width":W,"height":H,"Red":[u,v],"Yellow":null,"Blue":[u,v]}
        std::ostringstream json;
        json << "{\"width\":" << img_w << ",\"height\":" << img_h;
        for (auto & color : colors_) {
            json << ",\"" << color.name << "\":";
            auto it = detections.find(color.name);
            if (it != detections.end()) {
                json << "[" << (int)it->second.x << "," << (int)it->second.y << "]";
            } else {
                json << "null";
            }
        }
        json << "}";

        std_msgs::msg::String det_msg;
        det_msg.data = json.str();
        det_pub_->publish(det_msg);

        // Tuning window
        if (tune_mode_) {
            cv::Mat tune_mask;
            cv::inRange(hsv,
                cv::Scalar(tune_.h_low, tune_.s_low, tune_.v_low),
                cv::Scalar(tune_.h_high, tune_.s_high, tune_.v_high),
                tune_mask);
            cv::Mat cam_small, mask_bgr, mask_small;
            cv::resize(frame, cam_small, cv::Size(640, 480));
            cv::cvtColor(tune_mask, mask_bgr, cv::COLOR_GRAY2BGR);
            cv::resize(mask_bgr, mask_small, cv::Size(640, 480));
            cv::Mat view;
            cv::hconcat(cam_small, mask_small, view);
            cv::putText(view, "Camera", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255,255,255}, 2);
            cv::putText(view, "Mask (white=detected)", {660, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255,255,255}, 2);
            cv::putText(view, "Press S to save", {660, 460}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,255,255}, 2);
            cv::imshow(tune_win_, view);
            int key = cv::waitKey(1);
            if (key == 's' || key == 'S') saveParams();
        }

        img_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg());
    }

    cv::Mat buildMask(const cv::Mat & hsv, const ColorRange & color)
    {
        cv::Mat mask;
        cv::inRange(hsv, color.lower1, color.upper1, mask);
        if (color.dual) {
            cv::Mat mask2;
            cv::inRange(hsv, color.lower2, color.upper2, mask2);
            cv::bitwise_or(mask, mask2, mask);
        }
        return mask;
    }

    cv::Mat cleanMask(const cv::Mat & mask)
    {
        cv::Mat result;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask,   result, cv::MORPH_OPEN,  kernel);
        cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel);
        return result;
    }

    // Returns the centroid of the largest valid square contour, or nullopt.
    std::optional<cv::Point2f> detectBest(cv::Mat & frame,
                                           const cv::Mat & mask,
                                           const ColorRange & color)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double best_area = 0;
        std::optional<cv::Point2f> best_center;

        for (auto & contour : contours) {
            double area = cv::contourArea(contour);
            if (area < min_area_) continue;

            std::vector<cv::Point> approx;
            double peri = cv::arcLength(contour, true);
            cv::approxPolyDP(contour, approx, approx_eps_ * peri, true);
            if (approx.size() != 4 || !cv::isContourConvex(approx)) continue;

            cv::RotatedRect rrect = cv::minAreaRect(contour);
            float w = rrect.size.width, h = rrect.size.height;
            if (w == 0 || h == 0) continue;
            float aspect = (w > h) ? w / h : h / w;
            if (aspect < aspect_min_ || aspect > aspect_max_) continue;

            double sides[4];
            for (int i = 0; i < 4; i++) {
                cv::Point2f d = approx[i] - approx[(i + 1) % 4];
                sides[i] = std::sqrt(d.x * d.x + d.y * d.y);
            }
            double minS = *std::min_element(sides, sides + 4);
            double maxS = *std::max_element(sides, sides + 4);
            if (maxS == 0 || (minS / maxS) < side_ratio_min_) continue;

            if (area > best_area) {
                best_area = area;
                best_center = rrect.center;
            }

            // Draw
            cv::Point2f pts[4];
            rrect.points(pts);
            for (int i = 0; i < 4; i++)
                cv::line(frame, pts[i], pts[(i+1)%4], color.drawColor, 3);

            cv::Rect bbox = cv::boundingRect(approx);
            std::string label = color.name;
            int baseline = 0;
            cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.65, 2, &baseline);
            cv::Point tp = bbox.tl() + cv::Point(5, -10);
            cv::rectangle(frame, tp + cv::Point(0, baseline),
                tp + cv::Point(ts.width, -ts.height - 4), cv::Scalar(0,0,0), cv::FILLED);
            cv::putText(frame, label, tp, cv::FONT_HERSHEY_SIMPLEX, 0.65, color.drawColor, 2);
        }

        if (best_center.has_value()) {
            RCLCPP_DEBUG(this->get_logger(), "Detected %s at pixel (%.0f, %.0f)",
                color.name.c_str(), best_center->x, best_center->y);
        }
        return best_center;
    }

    void saveParams()
    {
        if (tune_idx_ >= 0) {
            auto & c = colors_[tune_idx_];
            c.lower1 = cv::Scalar(tune_.h_low, tune_.s_low, tune_.v_low);
            c.upper1 = cv::Scalar(tune_.h_high, tune_.s_high, tune_.v_high);
        }
        std::ofstream f(params_file_);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not write to %s", params_file_.c_str());
            return;
        }
        f << "color_square_detector:\n  ros__parameters:\n";
        for (auto & c : colors_) {
            std::string key = c.name;
            std::transform(key.begin(), key.end(), key.begin(), ::tolower);
            f << "    " << key << "_h_low:  " << (int)c.lower1[0] << "\n";
            f << "    " << key << "_h_high: " << (int)c.upper1[0] << "\n";
            f << "    " << key << "_s_low:  " << (int)c.lower1[1] << "\n";
            f << "    " << key << "_s_high: " << (int)c.upper1[1] << "\n";
            f << "    " << key << "_v_low:  " << (int)c.lower1[2] << "\n";
            f << "    " << key << "_v_high: " << (int)c.upper1[2] << "\n";
            if (c.dual) {
                f << "    " << key << "_h_low2:  " << (int)c.lower2[0] << "\n";
                f << "    " << key << "_h_high2: " << (int)c.upper2[0] << "\n";
                f << "    " << key << "_s_low2:  " << (int)c.lower2[1] << "\n";
                f << "    " << key << "_s_high2: " << (int)c.upper2[1] << "\n";
                f << "    " << key << "_v_low2:  " << (int)c.lower2[2] << "\n";
                f << "    " << key << "_v_high2: " << (int)c.upper2[2] << "\n";
            }
        }
        RCLCPP_INFO(this->get_logger(), "Saved HSV params to: %s", params_file_.c_str());
    }

    void loadParams()
    {
        if (!std::filesystem::exists(params_file_)) return;
        for (auto & c : colors_) {
            std::string key = c.name;
            std::transform(key.begin(), key.end(), key.begin(), ::tolower);
            auto decl = [&](const std::string & p, int def) -> int {
                if (!this->has_parameter(p)) this->declare_parameter(p, def);
                return this->get_parameter(p).as_int();
            };
            c.lower1 = cv::Scalar(decl(key+"_h_low",(int)c.lower1[0]), decl(key+"_s_low",(int)c.lower1[1]), decl(key+"_v_low",(int)c.lower1[2]));
            c.upper1 = cv::Scalar(decl(key+"_h_high",(int)c.upper1[0]), decl(key+"_s_high",(int)c.upper1[1]), decl(key+"_v_high",(int)c.upper1[2]));
            if (c.dual) {
                c.lower2 = cv::Scalar(decl(key+"_h_low2",(int)c.lower2[0]), decl(key+"_s_low2",(int)c.lower2[1]), decl(key+"_v_low2",(int)c.lower2[2]));
                c.upper2 = cv::Scalar(decl(key+"_h_high2",(int)c.upper2[0]), decl(key+"_s_high2",(int)c.upper2[1]), decl(key+"_v_high2",(int)c.upper2[2]));
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded HSV params from %s", params_file_.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    img_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      det_pub_;

    std::vector<ColorRange> colors_;
    TuneState               tune_;
    int                     tune_idx_ = -1;
    std::string             tune_win_;
    std::string             tune_color_;
    std::string             params_file_;

    int    min_area_;
    double approx_eps_;
    double side_ratio_min_;
    double aspect_min_;
    double aspect_max_;
    bool   tune_mode_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorSquareDetector>());
    rclcpp::shutdown();
    return 0;
}
