using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.JacoUnity;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using System.Collections.Generic;
using System.IO;

public class GarbageSorting : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 1f;
    const float k_Angle = 40f;		// for closing gripper fingers

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "jaco_unity";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_j2n6s200;
    public GameObject j2n6s200 { get => m_j2n6s200; set => m_j2n6s200 = value; }
    
    // Pick Default Values
    private Quaternion PickOrientation;
    private Quaternion DefaultPickOrientation = new Quaternion(0.6935244202613831f, -0.02997758984565735f, 0.716662585735321f, -0.06723421812057495f);
    private Quaternion VerticalPickOrientation = new Quaternion(-0.02454143762588501f, 0.07347844541072846f, -0.9966778755187988f, 0.025146296247839929f);
    readonly Vector3 PickPosition = new Vector3(0f, 0.4f, -0.5f);
    
    // Place Default Values
    readonly Quaternion PlaceOrientation = new Quaternion(0.19721326231956483f, -0.0665614977478981f, 0.9772276878356934f, 0.04126504436135292f);
    private Vector3 PlacePosition = new Vector3(-0.569f, 0.023f, -0.130f);
    
    // For camera
    public Camera targetCamera;
    private string savePath = "temp_txt/";
    private string fileNamePrefix = "screenshot";
    
    // For raycast
    public LayerMask raycastMask = -1;
    public Transform raycastOrigin; // Reference to a child GameObject acting as the raycast origin
    
    // For making the Target child to the end effector
    [SerializeField]
    private Transform m_endEffector;
    
    // Object Center Coordinates
    private string filePathCoordinates = string.Concat('/', Directory.GetCurrentDirectory(), "/temp_txt/coordinates.txt");
    
    // For recognising detection
    private string filePathDetection = string.Concat('/', Directory.GetCurrentDirectory(), "/temp_txt/detection.txt");
	
    // For distance
    private string filePathDistance = string.Concat('/', Directory.GetCurrentDirectory(), "/temp_txt/distance.txt");
    
    // For rotation
    private string filePathRotate = string.Concat('/', Directory.GetCurrentDirectory(), "/temp_txt/rotate.txt");
    
    // For choosing bin
    private string filePathCategory = string.Concat('/', Directory.GetCurrentDirectory(), "/temp_txt/category.txt");


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
        
        // Introduce a delay before calling DetectionPipeline()
        yield return new WaitForSeconds(5.0f);
        
        StartCoroutine(DetectionPipeline());
    }
    
    /// <summary>
    ///     Execute the whole pipeline containing 4 stages:
    ///         1. Start            Move the robot above the table and take a photo
    ///         2. Pre Grasp        Position gripper directly above target object and measure the vertical destance using raycast sensor
    ///         3. Go Down          Lower gripper so that fingers are on either side of object
    ///         4. Grasp And Up     Close gripper and move the end-effector a little upwards
    ///         5. Place            Move the end-effector to the place position and open the gripper
    /// </summary>
    IEnumerator DetectionPipeline()
    {
        while(true)
        {
            PickOrientation = DefaultPickOrientation;

            stage = (int)Poses.Start;
	        PublishJoints();
	        Debug.Log("Starting Coroutine ScreenshotPosition.");      
	        yield return new WaitForSeconds(10.0f); // Waiting for ML model's response
	        
	        string detection = ReadLineFromFile(filePathDetection);
	        if (detection=="False"){
	            Debug.Log("No detecttion.");  
	            continue;}
	        
	        string category = ReadLineFromFile(filePathCategory);
	        Debug.Log("Object category: " + category);
	        
	        if (category=="metal")	// blue
	            PlacePosition = new Vector3(-0.518f, 0.004f, 0.282f);
	        else if (category=="plastic")	// red
	            PlacePosition = new Vector3(0.0f, 0.004f, 0.282f);
	        else if (category=="paper")	// green
	            PlacePosition = new Vector3(0.532f, 0.004f, 0.282f);
	        else if (category=="glass")	// yellow
	            PlacePosition = new Vector3(0.526f, 0.004f, -0.149f);
	        else if (category=="other")	// grey
	            PlacePosition = new Vector3(-0.569f, 0.023f, -0.130f);
	            
	        stage = (int)Poses.PreGrasp;
	        PublishJoints();
	        Debug.Log("Starting Coroutine Pregrasp.");
	        yield return new WaitForSeconds(6.0f); // Delay Start->Pregrasp
	 
	        // If the rotation of the gripper is wrong correct it
            string line = ReadLineFromFile(filePathRotate); 
            if (line == "True")
            {
                PickOrientation = VerticalPickOrientation;
            }
	        
	        // Raycast sensor
            if (raycastOrigin == null)
            {
                Debug.LogError("Raycast origin (child GameObject) is not assigned!");
            }

            // Cast a ray from the raycast origin's position towards its forward direction
            Ray ray = new Ray(raycastOrigin.position, raycastOrigin.forward);
            RaycastHit hit;

            // Check if the ray hits an object on the specified layer
            if (Physics.Raycast(ray, out hit, Mathf.Infinity, raycastMask))
            {
                // Get the distance between the raycast origin and the hit object
                float distance = Vector3.Distance(raycastOrigin.position, hit.point);
                 
                // Draw the ray from the origin to the hit point
                Debug.DrawRay(ray.origin, ray.direction * distance, Color.red, 2f);

                // You can now use the 'distance' value as needed (e.g., print it, display it in UI, etc.)
                Debug.Log("Distance from camera center to object: " + distance);
                
                // Write the distance to the .txt file
                WriteDistanceToFile(distance, filePathDistance);
            }
            else
            {
                Debug.Log("Raycast did not hit any object.");
            }
            // yield return new WaitForSeconds(1f);

                
            // GoDown Pose
            stage = (int)Poses.GoDown;
            PublishJoints();
            Debug.Log("Starting Coroutine GoDown..Grasp..Place.");
            yield return new WaitForSeconds(15.0f);  // Delay Pregasp-> GoDown -> GraspAndUp -> Place
	    }
    }
    
    
    /// <summary>
    ///     Write the distance measured by the raycast sensor in a .txt file
    /// </summary>
    private void WriteDistanceToFile(float distance, string filePath)
    {
        using (StreamWriter writer = new StreamWriter(filePath, false))
        {
            writer.WriteLine(distance);
            writer.WriteLine("0");
        }
    }
    
    /// <summary>
    ///     Read the first line of a .txt file
    /// </summary>
    private string ReadLineFromFile(string filePath) 
    {
        using (StreamReader reader = new StreamReader(filePath)) 
        {
            string line = reader.ReadLine(); 
            return line; 
        } 
    }
     
    /// <summary>
    ///     Take a photo from the RobotCamera attached to the end-effector, and store it as a png file.
    /// </summary>
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
     
            //Debug.Log("Screenshot saved to: " + filePath);
        }
        else
        {
            Debug.LogError("Target camera is not assigned!");
        }
    }

    /// <summary>
    ///     Close gripper fingers.
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = k_Angle;
        rightDrive.target = k_Angle;
        
        leftDrive.stiffness = 4000;
        rightDrive.stiffness = 4000;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }
    
    /// <summary>
    ///     Open gripper fingers.
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -k_Angle;
        rightDrive.target = -k_Angle;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    JacoMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new JacoMoveitJointsMsg();

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
           position = PickPosition.To<FLU>(),
           orientation = PickOrientation.To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = PlacePosition.To<FLU>(),
            orientation = PlaceOrientation.To<FLU>()
        };

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {

        if (response.trajectories.Length > 0)
        {
            //Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
            // Write 0, 0 to the .txt file
            float distance = 0;
            WriteDistanceToFile(distance, filePathCoordinates);
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     ExecStartCoroutine(ExecuteTrajectories(response));uting a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {        
        if (response.trajectories != null)
        {     
            int stopIndex = (stage == (int)Poses.GoDown) ? 5 : stage+1;

            // For every trajectory plan returned
            for (var poseIndex = stage; poseIndex < stopIndex; poseIndex++)
            {

                // Close the gripper
                if (poseIndex == (int)Poses.GraspAndUp)
                {
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
                }

                // Open the gripper
                if (poseIndex == (int)Poses.Place)
                {
                    yield return new WaitForSeconds(k_PoseAssignmentWait);
                    OpenGripper();
                    yield return new WaitForSeconds(k_PoseAssignmentWait);
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }
        }
    }

    enum Poses
    {
        Start,
        PreGrasp,
        GoDown,
        GraspAndUp,
        Place
    }
}
