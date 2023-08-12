using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MakeChild : MonoBehaviour
{

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
        
        if(other.gameObject.tag == "TargetPlacement")
        {
            StartCoroutine(WaitBeforeOpen());
        }
    }
    
}
