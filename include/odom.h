
class Odom
{
  private:
  public:
    double verticalOdoPos, horizontalOdoPos;
    double xPos, yPos, orientation;
    void setInitPos(double xPos, double yPos, double orientation, double verticalOdoPos, double horizontalOdoPos);
    void update(double verticalOdoPos, double horizontalOdoPos, double orientation);
};
