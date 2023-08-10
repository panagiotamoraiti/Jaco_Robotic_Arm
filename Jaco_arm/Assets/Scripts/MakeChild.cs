using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MakeChild : MonoBehaviour
{

    Transform targetTransform;
    
    IEnumerator WaitBeforeOpen()
    {
        yield return new WaitForSeconds(1.5f);
    }

    void OnTriggerEnter (Collider other)
    {
        if(other.gameObject.tag == "Target")
        {
            other.gameObject.GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezeAll;
            other.transform.parent = transform;
            targetTransform = other.transform;
        }
        
        if(other.gameObject.tag == "TargetPlacement")
        {
          
            StartCoroutine(WaitBeforeOpen());
            //Transform targetTransform = transform.FindWithTag("Target");
            if(targetTransform.transform.parent != null)
            {
                targetTransform.transform.parent = null;
            }
            
            // Unfreeze position and rotation
            targetTransform.gameObject.GetComponent<Rigidbody>().constraints = RigidbodyConstraints.None;
        }
    }
    
}
