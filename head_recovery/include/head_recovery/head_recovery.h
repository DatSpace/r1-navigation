#ifndef HEAD_RECOVERY_HEAD_RECOVERY_H
#define HEAD_RECOVERY_HEAD_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

namespace head_recovery
{
  /**
   * @class HeadRecovery
   * @brief A recovery behavior that rotates the robot in-place to attempt to clear out space
   */
  class HeadRecovery : public nav_core::RecoveryBehavior
  {
  public:
    /**
     * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
     */
    HeadRecovery();

    /**
     * @brief  Initialization function for the HeadRecovery recovery behavior
     * @param name Namespace used in initialization
     * @param tf (unused)
     * @param global_costmap (unused)
     * @param local_costmap (unused)
    */
    void initialize(std::string name, tf2_ros::Buffer *,
                    costmap_2d::Costmap2DROS *, costmap_2d::Costmap2DROS *local_costmap);

    /**
     * @brief  Run the HeadRecovery recovery behavior.
     */
    void runBehavior();

    /**
     * @brief  Destructor for the head recovery behavior
     */
    ~HeadRecovery();

  private:
    bool initialized_;
    float movement_coords_[4][2];
    std::vector<std::string> yarp_ports_;
    ros::Publisher pub_;
  };
};     // namespace head_recovery
#endif // HEAD_RECOVERY_HEAD_RECOVERY_H
