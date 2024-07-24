// Ensure that is not defined multiple times
#ifndef M02_BASE_FIGURE_BASE_FIGURE_HPP
#define M02_BASE_FIGURE_BASE_FIGURE_HPP

namespace m02_base_figure
{
    // Parent class of the plugin
    class BaseFigure
    {
        public:
            // Initialize most valuable size, must create impl. with child
            virtual void initialize(double side_length) = 0;

            // Calculate area, must create implementation with child
            virtual double area() = 0;

            // Destructor
            virtual ~BaseFigure(){}

        protected:
            // Protected constructor as it will be used with inheritance
            BaseFigure(){}
    };
}  // namespace m02_base_figure

#endif  // M02_BASE_FIGURE_BASE_FIGURE_HPP