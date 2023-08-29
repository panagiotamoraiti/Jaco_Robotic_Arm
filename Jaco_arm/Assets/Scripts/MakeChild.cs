using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;

/*
    This class is used to simulate grasping and letting go of an object.
    The object becomes child to the gripper when the robot is supposed to be holding it.
*/

public class MakeChild : MonoBehaviour
{
    private string filePathCategory = string.Concat('/', Directory.GetCurrentDirectory(), "/temp_txt/category.txt");

    private string ReadLineFromFile(string filePath) 
    { 
        /* Read the first line from the .txt file */
        using (StreamReader reader = new StreamReader(filePath)) 
        { 
        string line = reader.ReadLine(); 
        return line; 
        } 
    }

    Transform targetTransform;

    IEnumerator WaitBeforeOpen()
    {
        /* This is needed for synchronizing the "unchilding" of the object with the openning of the gripper */
        yield return new WaitForSeconds(1.2f);
        targetTransform.parent = null;
        targetTransform.gameObject.GetComponent<Rigidbody>().constraints = RigidbodyConstraints.None;
    }

    void OnTriggerEnter (Collider other)
    {
        /* If the end-effector collides with an object tagged "Target", it becomes a parent of this object */
        if(other.gameObject.tag == "Target")
        {
            other.gameObject.GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezeAll;
            other.transform.parent = transform;
            targetTransform = other.transform;
        }

        /* If the end-effector collides with one of the colliders above the bins, 
           the child object of the end-effector is released */
        string category = ReadLineFromFile(filePathCategory);

        /* Check if the category of the object matches the bin
           This prevents placing the object in the wrong bin */
        bool unchild = (other.gameObject.tag=="PlasticBin" && category=="plastic"
               || other.gameObject.tag=="MetalBin" && category=="metal"
               || other.gameObject.tag=="PaperBin" && category=="paper"
               || other.gameObject.tag=="GlassBin" && category=="glass"
               || other.gameObject.tag=="GarbageBin" && category=="other");

        if(unchild)
        {
            StartCoroutine(WaitBeforeOpen());
        }
    }
}
