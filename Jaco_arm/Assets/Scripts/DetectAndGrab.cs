using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class DetectAndGrab : MonoBehaviour
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
    // [SerializeField]
    // GameObject m_Target;
    // public GameObject Target { get => m_Target; set => m_Target = value; }
    //[SerializeField]
    //GameObject m_TargetPlacement;
    //public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

    readonly Quaternion m_PickOrientation = new Quaternion(0.19721326231956483f, -0.0665614977478981f, 0.9772276878356934f, 0.04126504436135292f);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;
    Quaternion or = new Quaternion(0.6935244202613831f, -0.02997758984565735f, 0.716662585735321f, -0.06723421812057495f);
    // Vector3 pos = new Vector3(-0.047462593764066699f, 0.3511176109313965f, -0.47372329235076907f);
    Vector3 pos = new Vector3(0f, 0.45f, -0.5f);
    
    //UnityEditor.TransformWorldPlacementJSON:{"position":{"x":-0.047462593764066699,"y":0.3511176109313965,"z":-0.47372329235076907},"rotation":{"x":0.6935244202613831,"y":-0.02997758984565735,"z":0.716662585735321,"w":-0.06723421812057495},"scale":{"x":1.0,"y":1.0,"z":1.0}}
    
    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;
    
    // Start is called before the first frame update
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
    
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 60f;
        rightDrive.target = 60f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -60f;
        rightDrive.target = -60f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }
    
    
    NiryoMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new NiryoMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

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
        /*request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.localPosition + m_PickPoseOffset).To<FLU>(),
	        orientation = m_PickOrientation.To<FLU>()
        };*/


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
