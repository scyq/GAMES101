#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
                  << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1)
    {
        return control_points[0];
    }
    std::vector<cv::Point2f> new_ctrl_points;
    // if we know A and B, we want to know C in AB
    // v_AB = B - A
    // v_AC = C - A = t(v_AB)
    // C = v_AC + A
    //   = t(B - A) + A
    //   = tB + (1 - t)A
    for (int i = 0; i < control_points.size() - 1; i++)
    {
        float x = t * control_points[i].x + (1 - t) * control_points[i + 1].x;
        float y = t * control_points[i].y + (1 - t) * control_points[i + 1].y;
        new_ctrl_points.emplace_back(x, y);
    }
    return recursive_bezier(new_ctrl_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.0001)
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        // anti-aliasing
        std::vector<cv::Point2f> neighbors;
        float floor_x = floor(point.x);
        float ceil_x = ceil(point.x);
        float floor_y = floor(point.y);
        float ceil_y = ceil(point.y);

        neighbors.emplace_back(floor_x, point.y);
        neighbors.emplace_back(point.x, floor_y);
        neighbors.emplace_back(ceil_x, point.y);
        neighbors.emplace_back(point.x, ceil_y);

        for (auto neighbor : neighbors)
        {
            float distance = sqrt(pow(point.x - neighbor.x, 2) + pow(point.y - neighbor.y, 2));
            window.at<cv::Vec3b>(neighbor.y, neighbor.x)[1] = std::max(255 * (1 - distance), (float)window.at<cv::Vec3b>(neighbor.y, neighbor.x)[1]);
        }
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27)
    {
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4)
        {
            // naive_bezier(control_points, window); // standard bezier
            bezier(control_points, window); // my bezier

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
