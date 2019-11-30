#include "path.hpp"
#include <vector>
#include <functional>

namespace path
{
class Bezier : public Path
{
private:
  std::vector<Point> points;

  double combination(int n, int r);

public:
  Bezier(std::vector<Point> points, int resolution, int lookahead = -1);

  Point pointAt(int T);
};
} // namespace path
