// ------------------------ C++ STANDARD LIBRARIES ----------------------------
#include <cmath>

// ------------------------ HEADER FROM BASE CLASS ----------------------------
#include "m02_base_figure/base_figure.hpp"

namespace m02_figure_plugins
{
    /**
     * Class that inheritates from BaseFigure to define a square, a 2D figure
     * with two sides of the same lenght, so the area is the square of the 
     * side.
    */
    class Square : public m02_base_figure::BaseFigure
    {
    public:
        void initialize(double side_length) override
        {
            side_length_ = side_length;
        }

        double area() override
        {
            return pow(side_lenght_, 2);
        }
    protected:
        double side_length_;
    };

    /**
     * Class that inheritates from BaseFigure to define a equlateral triangle, 
     * a 2D figure with three sides of the same lenght, so the area is the 
     * square of the side.
    */
    class Triangle : public m02_base_figure::BaseFigure
    {
    public:
        void initialize(double side_length) override
        {
            side_length_ = side_length;
        }

        double area() override
        {
            return 0.5 * side_length_ * getHeight();
        }

        double getHeight()
        {
            return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * 
                (side_length_ / 2)));
        }
    protected:
        double side_length_;
    };

    /**
     * Class that inheritates from BaseFigure to define a circle, 
     * a 2D figure with a radius of the same lenght distributed 360°, 
     * so the area is the square of the radius and the factor PI.
    */
    class Circle : public m02_base_figure::BaseFigure
    {
    public:
        void initialize(double side_length) override
        {
            side_length_ = side_length;
        }

        double area() override
        {
            return M_PI * side_length_ * side_length_;
        }
    protected:
        double side_length_;
    };
}  // namespace m02_figure_plugins

// Export the classes as plugins by using the macro of plugin lib
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(m02_figure_plugins::Square, m02_base_figure::BaseFigure)
PLUGINLIB_EXPORT_CLASS(m02_figure_plugins::Triangle, m02_base_figure::BaseFigure)
PLUGINLIB_EXPORT_CLASS(m02_figure_plugins::Circle, m02_base_figure::BaseFigure)
