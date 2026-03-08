

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_MODEL_PLUGIN_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_MODEL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h> 
#include <nezha_plugins/SetWaveParameters.h>
#include <memory>

namespace asv
{
  
///////////////////////////////////////////////////////////////////////////////
// WavefieldModelPlugin

  /// \internal
  /// \brief Class to hold private data for WavefieldModelPlugin.
  class WavefieldModelPluginPrivate;

  /// \brief A Gazebo model plugin to simulate water waves.
  ///
  /// # Usage
  /// 
  /// Add the SDF for the plugin to the <model> element of your wave model. 
  ///
  /// \code
  /// <plugin name="wavefield" filename="libWavefieldModelPlugin.so">
  ///   <static>false</static>
  ///   <update_rate>30</update_rate>
  ///   <size>1000 1000</size>
  ///   <cell_count>50 50</cell_count>
  ///   <wave>
  ///     <number>3</number>
  ///     <scale>1.5</scale>
  ///     <angle>0.4</angle>
  ///     <steepness>1.0</steepness>
  ///     <amplitude>0.4</amplitude>
  ///     <period>8.0</period>
  ///     <direction>1 1</direction>
  ///   </wave>
  ///   <markers>
  ///     <wave_patch>false</wave_patch>
  ///     <wave_patch_size>4 4</wave_patch_size>
  ///   </markers>
  /// </plugin>
  /// \endcode
  ///
  /// # Subscribed Topics
  ///
  /// 1. ~/request (gazebo::msgs::Request)
  ///   
  /// 2. ~/wave (gazebo::msgs::Param_V)
  ///
  /// # Published Topics
  ///
  /// 1. ~/reponse (gazebo::msgs::Response)
  ///
  /// 2. /marker (ignition::msgs::Marker)
  ///
  /// # Parameters
  ///
  /// 1. <static> (bool, default: false)
  ///   Create a static wave field if set to true.
  ///   
  /// 2. <update_rate> (double, default: 30.0)
  ///   The rate in Hz at which the wavefield is updated.
  ///   
  /// 3. <size> (Vector2D, default: (1000 1000))
  ///   A two component vector for the size of the wave field in each direction.
  ///   
  /// 4. <cell_count> (int, default: (50 50))
  ///   A two component vector for the number of grid cells in each direction.
  ///
  /// 5. <number> (int, default: 1)
  ///   The number of component waves.
  ///
  /// 6. <scale> (double, default: 2.0)
  ///   The scale between the mean and largest / smallest component waves.
  ///
  /// 7. <angle> (double, default: 2*pi/10)
  ///   The angle between the mean wave direction and the largest / smallest component waves.
  ///
  /// 8. <steepness> (double, default: 1.0)
  ///   A parameter in [0, 1] controlling the wave steepness with 1 being steepest.
  ///
  /// 9. <amplitude> (double, default: 0.0)
  ///   The amplitude of the mean wave in [m].
  ///
  /// 10. <period> (double, default: 1.0)
  ///   The period of the mean wave in [s].
  ///
  /// 11. <phase> (double, default: 0.0)
  ///   The phase of the mean wave.
  ///
  /// 12. <direction> (Vector2D, default: (1 0))
  ///   A two component vector specifiying the direction of the mean wave.
  ///
  /// 13. <wave_patch> (bool, default: false)
  ///   Display a wave patch marker if set to true.
  ///
  /// 14. <wave_patch_size> (Vector2D, default: (4 4))
  ///   A two component vector for the size of the wave marker (in units of the wave grid).
  ///
  class GAZEBO_VISIBLE WavefieldModelPlugin : public gazebo::ModelPlugin
  {
    /// \brief Destructor.
    public: virtual ~WavefieldModelPlugin();

    /// \brief Constructor.
    public: WavefieldModelPlugin();

    // Documentation inherited.
    public: void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: void Init();

    // Documentation inherited.
    public: void Fini();

    // Documentation inherited.
    public: void Reset();

    /// internal
    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// internal
    /// \brief Callback for gztopic "~/request" when the request is "wave_param".
    ///
    /// \param[in] _msg Request message.
    private: void OnRequest(ConstRequestPtr &_msg);
    ///
    /// \param[in] _msg Wave message.
    private: void OnWaveMsg(ConstParam_VPtr &_msg);
private: std::unique_ptr<ros::NodeHandle> ros_node_;
private: ros::ServiceServer srv_;

public: bool SetWaveParamsCallback(nezha_plugins::SetWaveParameters::Request &req,
                                   nezha_plugins::SetWaveParameters::Response &res);
    /// \internal
    /// \brief Private methods for managing markers.
    private: void InitMarker();
    private: void FiniMarker();
    private: void ResetMarker();
    private: void UpdateMarker();

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldModelPluginPrivate> data;
  };
} // namespace asv

#endif // nezha_plugins
