#include "m02_base_figure/base_figure.hpp"

namespace plugins_figure
{

    class Square : public polygon_base::BaseFigure
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

    class Triangle : public polygon_base::BaseFigure
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
    
    class Circle : public polygon_base::BaseFigure
    {
        public:
        void initialize(double side_length) override
        {
            side_length_ = side_length;
        }

        double area() override
        {
            return 3.1416 * side_length_ * side_length_;
        }
    };

}  // namespace m02_plugins_figure

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(plugins_figure::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(plugins_figure::Triangle, polygon_base::RegularPolygon)
