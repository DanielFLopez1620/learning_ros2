#ifndef M02_POLYGON_BASE_BASE_FIGURE_HPP
#define M02_POLYGON_BASE_BASE_FIGURE_HPP

namespace polygon_base
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