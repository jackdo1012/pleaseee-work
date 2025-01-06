
class Odom
{
  private:
    double verticalOdoPos, horizontalOdoPos;

  public:
    double xPos, yPos, orientation;
    void setInitPos(double xPos, double yPos, double orientation, double verticalOdoPos, double horizontalOdoPos);
    void update(double verticalOdoPos, double horizontalOdoPos, double orientation);
};