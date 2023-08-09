using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class raycast : MonoBehaviour
{
    public LayerMask raycastMask = -1;
    public Transform raycastOrigin; // Reference to a child GameObject acting as the raycast origin

    void Update()
    {
        // Check if the user presses a key (you can use any trigger event)
        if (Input.GetKeyDown(KeyCode.Space))
        {
            if (raycastOrigin == null)
            {
                Debug.LogError("Raycast origin (child GameObject) is not assigned!");
                return;
            }

            // Cast a ray from the raycast origin's position towards its forward direction
            Ray ray = new Ray(raycastOrigin.position, raycastOrigin.forward);
            RaycastHit hit;

            //Debug.DrawRay(ray.origin, ray.direction * 10f, Color.red, 2f); // Draw the ray for 2 seconds

            // Check if the ray hits an object on the specified layer
            if (Physics.Raycast(ray, out hit, Mathf.Infinity, raycastMask))
            {
                // Get the distance between the raycast origin and the hit object
                float distance = Vector3.Distance(raycastOrigin.position, hit.point);
                 
                // Draw the ray from the origin to the hit point
                Debug.DrawRay(ray.origin, ray.direction * distance, Color.red, 2f);

                // You can now use the 'distance' value as needed (e.g., print it, display it in UI, etc.)
                Debug.Log("Distance from camera center to object: " + distance);
            }
            else
            {
                Debug.Log("Raycast did not hit any object.");
            }
        }
    }
}

