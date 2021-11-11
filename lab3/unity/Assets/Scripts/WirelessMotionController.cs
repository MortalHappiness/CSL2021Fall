using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

public class WirelessMotionController : MonoBehaviour
{

    const string hostIP = "192.168.128.1";
    const int port = 80;

    private SocketClient socketClient;

    public bool isTrigger;
    public Quaternion quaternion;


    private void Awake() {
        socketClient = new SocketClient(hostIP, port);
    }

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        isTrigger = socketClient.isTrigger;
        quaternion = new Quaternion(socketClient.x, socketClient.y, socketClient.z, socketClient.w);
    }

    void OnDestroy () {
        socketClient.Close();
    }
}
