using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Enemy : MonoBehaviour
{
    public float speed;
    public Transform target;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        this.transform.LookAt(target);
        this.transform.position = this.transform.position + this.transform.forward * speed * Time.deltaTime;
    }

    public void OnCollideWithBullet()
    {
        Debug.Log(this.gameObject.name + "was killed");
        Destroy(this.gameObject);
    }
}
