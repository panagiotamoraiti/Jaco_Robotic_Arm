using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using System.Collections.Generic;
using System.IO;

public class CameraDetection : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 1.5f;
    const float k_PoseAssignmentWaitnew = 1.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "niryo_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_j2n6s200;
    public GameObject j2n6s200 { get => m_j2n6s200; set => m_j2n6s200 = value; }
    readonly Quaternion m_PickOrientation = new Quaternion(0.19721326231956483f, -0.0665614977478981f, 0.9772276878356934f, 0.04126504436135292f);
    // readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;
    Vector3 box_place = new Vector3(0.315f, 0.4f, 0.121f);
    
    // The hardcoded x/y/z angles assure that the gripper is always positioned above the target cube before grasping.
    Quaternion or = new Quaternion(0.6935244202613831f, -0.02997758984565735f, 0.716662585735321f, -0.06723421812057495f);
    //readonly Quaternion or = new Quaternion.Euler(0,90,0);
    //Debug.Log(or.ToString());
    Vector3 pos = new Vector3(0f, 0.4f, -0.5f);
    
    // for camera
    public Camera targetCamera;
    public string savePath = "Screenshots/";
    public string fileNamePrefix = "screenshot";
    
    public int stage;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;


    IEnumerator Start()
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
        
        // Introduce a delay before calling PublishJoints()
        yield return new WaitForSeconds(5.0f);
        stage = (int)Poses.Start;
        PublishJoints();
        Debug.Log("Starting Coroutine ScreenshotPosition.");      
        yield return new WaitForSeconds(12.0f);
        
        stage = (int)Poses.PreGrasp;
        PublishJoints();
        Debug.Log("Starting Coroutine Pregrasp.");
        yield return new WaitForSeconds(5.0f);
        
        stage = (int)Poses.GoDown;
        PublishJoints();
        Debug.Log("Starting Coroutine GoDown.");
        yield return new WaitForSeconds(8.0f);
        
        stage = (int)Poses.GraspAndPlace;
        PublishJoints();
        Debug.Log("Starting Coroutine GraspAndPlace.");
        yield return new WaitForSeconds(5.0f);
    }
    
    void Update()
    {

    }
    
    
    private void CaptureScreen()
        {
        // Create the screenshot file path
        string filePath = savePath + fileNamePrefix + ".png";
     
        // Capture the screenshot from the target camera
        if (targetCamera != null)
        {
            int width = 600;
            int height = 600;
            RenderTexture rt = new RenderTexture(width, height, 24);
            targetCamera.targetTexture = rt;
            Texture2D screenshot = new Texture2D(width, height, TextureFormat.RGB24, false);
            targetCamera.Render();
            RenderTexture.active = rt;
            screenshot.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            targetCamera.targetTexture = null;
            RenderTexture.active = null;
            Destroy(rt);
     
            // Convert the Texture2D to bytes and save the image
            byte[] bytes = screenshot.EncodeToPNG();
            System.IO.File.WriteAllBytes(filePath, bytes);
     
            Debug.Log("Screenshot saved to: " + filePath);
        }
        else
        {
            Debug.LogError("Target camera is not assigned!");
        }
        }

    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 60f;
        rightDrive.target = 60f;
        
        leftDrive.stiffness = 4000;
        rightDrive.stiffness = 4000;

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

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
           position = pos.To<FLU>(),
           orientation = or.To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = box_place.To<FLU>(),
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
            for (var poseIndex = stage; poseIndex < stage+1; poseIndex++)
            {
                
                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.GraspAndPlace)
                {
                    yield return new WaitForSeconds(k_PoseAssignmentWait);
                    CloseGripper();
                    yield return new WaitForSeconds(k_PoseAssignmentWait);
                }
                
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
                
                if (poseIndex == (int)Poses.Start)
                {
                    CaptureScreen();
                    yield return new WaitForSeconds(k_PoseAssignmentWaitnew);
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
        Start,
        PreGrasp,
        GoDown,
        GraspAndPlace
    }
}
