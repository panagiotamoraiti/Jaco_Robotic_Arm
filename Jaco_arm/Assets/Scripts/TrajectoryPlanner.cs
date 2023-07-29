using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 1.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "niryo_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_j2n6s200;
    public GameObject j2n6s200 { get => m_j2n6s200; set => m_j2n6s200 = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    //readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    //readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;
    //readonly Quaternion m_PickOrientation = new Quaternion(0.197f, -0.067f, 0.977f, 0.041f);
    
    // readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Quaternion m_PickOrientation = new Quaternion(0.19721326231956483f, -0.0665614977478981f, 0.9772276878356934f, 0.04126504436135292f);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;
    // Quaternion or = new Quaternion(-0.72423f, 0.13094f, 0.60637f, -0.30111f);
    // Quaternion or = new Quaternion(0.07173144817352295f, -0.027387527748942376f, 0.996719479560852f, -0.025599028915166856f); // functional
    Quaternion or = new Quaternion(-0.003451f, 0.006689f, 0.989699f, -0.142966f);
    // Vector3 pos = new Vector3(0.10684f, 0.22462f, 0.8162f);

// UnityEditor.TransformWorldPlacementJSON:{"position":{"x":0.36590585112571719,"y":0.09689018875360489,"z":-0.15794116258621217},"rotation":{"x":-0.0034519752953201534,"y":0.006689317524433136,"z":0.989699125289917,"w":-0.14296650886535645},"scale":{"x":1.0,"y":1.0,"z":1.0}}  // pick orientation

// UnityEditor.TransformWorldPlacementJSON:{"position":{"x":0.18070322275161744,"y":0.12306038290262222,"z":-0.36768752336502077},"rotation":{"x":0.19721326231956483,"y":-0.0665614977478981,"z":0.9772276878356934,"w":0.04126504436135292},"scale":{"x":1.0,"y":1.0,"z":1.0}}  // place orientation

    //Quaternion or = new Quaternion(-0.0039f, 0.0066f, 0.9896f, -0.1434f);  // working in panagota's pc
    //Vector3 pos = new Vector3(0.43f, 0.1f, -0.05299999f);

    Vector3 pos; 
    Vector3 box_pos;
    void Awake()
    {
        // Initialize pos using m_Target's local position
        box_pos = m_Target.transform.localPosition; 
        pos.x = box_pos.x;
        pos.y = box_pos.y + 0.06f; 
        pos.z = box_pos.z;          
    } 
 
    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_j2n6s200.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Find left and right fingers
        var rightGripper = linkName + "/j2n6s200_link_finger_1";
        var leftGripper = linkName + "/j2n6s200_link_finger_2";

        m_RightGripper = m_j2n6s200.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_j2n6s200.transform.Find(leftGripper).GetComponent<ArticulationBody>();
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 60f;
        rightDrive.target = 60f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -60f;
        rightDrive.target = -60f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>NiryoMoveitJoints</returns>
    NiryoMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new NiryoMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        //Pick Pose
        request.pick_pose = new PoseMsg
        {
           //position = (m_Target.transform.localPosition + m_PickPoseOffset).To<FLU>(),
           position = pos.To<FLU>(),
           // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
           orientation = or.To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.localPosition + m_PickPoseOffset).To<FLU>(),
	    orientation = m_PickOrientation.To<FLU>()
        };

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from niryo_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    yield return new WaitForSeconds(k_PoseAssignmentWait);
                    CloseGripper();
                    //yield return new WaitForSeconds(k_PoseAssignmentWait);
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }
}
