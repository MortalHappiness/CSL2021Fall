using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
    public SimpleShoot gun;

    public float shootColdDownTime = 1f;
    private float shootColdDownTimer;

    WirelessMotionController wirelessMotionController;

    // Start is called before the first frame update
    void Start()
    {
        wirelessMotionController = this.GetComponent<WirelessMotionController>();
    }

    // Update is called once per frame
    void Update()
    {
        this.transform.rotation = wirelessMotionController.quaternion;

        if (shootColdDownTimer <= 0f && wirelessMotionController.isTrigger)
        {
            shootColdDownTimer = shootColdDownTime;
            gun.GetComponent<Animator>().SetTrigger("Fire");
        } else
        {
            shootColdDownTimer = shootColdDownTimer - Time.deltaTime;
        }
    }
}
