using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;

public class MakeChild : MonoBehaviour
{

    
    private string filePathCategory = "/home/beast1/Jaco_Robotic_Arm/Jaco_arm/Screenshots/category.txt";

    private string ReadLineFromFile(string filePath) 
    { 
        // Read the first line from the .txt file 
        using (StreamReader reader = new StreamReader(filePath)) 
        { 
        string line = reader.ReadLine(); 
        return line; 
        } 
    }

    Transform targetTransform;
    
    IEnumerator WaitBeforeOpen()
    {
        yield return new WaitForSeconds(1.2f);
        targetTransform.parent = null;
        targetTransform.gameObject.GetComponent<Rigidbody>().constraints = RigidbodyConstraints.None;
    }

    void OnTriggerEnter (Collider other)
    {
        if(other.gameObject.tag == "Target")
        {
            other.gameObject.GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezeAll;
            other.transform.parent = transform;
            targetTransform = other.transform;
        }
        
        string category = ReadLineFromFile(filePathCategory);
        bool unchild = (other.gameObject.tag=="PlasticBin" && category=="plastic"
               || other.gameObject.tag=="MetalBin" && category=="metal"
               || other.gameObject.tag=="GarbageBin" && category=="other");
        
        if(unchild)
        {
            StartCoroutine(WaitBeforeOpen());
        }
    }
    
}
