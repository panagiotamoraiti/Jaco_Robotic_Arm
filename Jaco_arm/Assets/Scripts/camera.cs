using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.IO;

/*
This script takes a snapshot with the Robot Camera (which is a child of link_6 of the robotic arm)/
*/
 
public class camera : MonoBehaviour
{
    public Camera targetCamera;
    public string savePath = "Screenshots/";
    public string fileNamePrefix = "screenshot";
 
    private float timer = 30f; // Time between captures
    private float currentTime = 0f;
 
    private void Update()
    {
        currentTime += Time.deltaTime;
        
        if (currentTime >= timer)
        {
            CaptureScreen();
            currentTime = 0f;
        }
    }
 
    private void CaptureScreen()
    {
        // Create the screenshot file path
        // string filePath = savePath + fileNamePrefix + "_" + System.DateTime.Now.ToString("yyyyMMddHHmmss") + ".png";
        string filePath = savePath + fileNamePrefix + ".png";
 
        // Capture the screenshot from the target camera
        if (targetCamera != null)
        {
            RenderTexture rt = new RenderTexture(Screen.width, Screen.height, 24);
            targetCamera.targetTexture = rt;
            Texture2D screenshot = new Texture2D(Screen.width, Screen.height, TextureFormat.RGB24, false);
            targetCamera.Render();
            RenderTexture.active = rt;
            screenshot.ReadPixels(new Rect(0, 0, Screen.width, Screen.height), 0, 0);
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
}
 
