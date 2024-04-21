#ifndef M02_BASE_FIGURE_BASE_FIGURE_HPP
#define M02_BASE_FIGURE_BASE_FIGURE_HPP

namespace base_figure
{
    class BaseFigure
    {
        public:
            virtual void initialize(double length_base_dim) = 0;
            virtual double area() = 0;
            virtual ~BaseFigure(){}
        protected:
            BaseFigure(){}
    };
}

#endif