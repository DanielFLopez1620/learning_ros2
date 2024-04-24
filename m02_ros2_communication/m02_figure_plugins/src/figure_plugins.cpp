#include "m02_base_figure/base_figure.hpp"
#include <cmath>

namespace m02_figure_plugins
{
    class Square : public figure::BaseFigure
    {
        public:
        void initialize(double side_length) override
        {
            side_length_ = side_length;
        }

        double area() override
        {
            return side_length_ * side_length_;
        }

        protected:
        double side_length_;
    };

    class Triangle : public figure::BaseFigure
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
            return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
        }

        protected:
        double side_length_;
    };

    class Circle : public figure::BaseFigure
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

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(m02_figure_plugins::Square, figure::BaseFigure)
PLUGINLIB_EXPORT_CLASS(m02_figure_plugins::Triangle, figure::BaseFigure)
