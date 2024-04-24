#ifndef M02_BASE_FIGURE_BASE_FIGURE_HPP
#define M02_BASE_FIGURE_BASE_FIGURE_HPP

namespace figure
{
    class BaseFigure
    {
        public:
        virtual void initialize(double side_length) = 0;
        virtual double area() = 0;
        virtual ~BaseFigure(){}

        protected:
        BaseFigure(){}
    };
}  // namespace figure

#endif  // M02_BASE_FIGURE_BASE_FIGURE_HPP