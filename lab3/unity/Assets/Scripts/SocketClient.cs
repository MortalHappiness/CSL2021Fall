using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public class SocketClient
{
    private Socket socketClient;
    private Thread thread;
    private byte[] data = new byte[1024];

    public bool isTrigger;
    public float x, y, z, w;

    private bool inited = false;
    private float x_init, y_init, z_init;

    public SocketClient(string hostIP, int port) {
        thread = new Thread(() => {
            // while the status is "Disconnect", this loop will keep trying to connect.
            while (true) {
                try {
                    socketClient = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                    socketClient.Connect(new IPEndPoint(IPAddress.Parse(hostIP), port));
                    // while the connection
                    while (true) {
                        /*********************************************************
                         * TODO: you need to modify receive function by yourself *
                         *********************************************************/
                        if (socketClient.Available < 100) {
                            Thread.Sleep(1);
                            continue;
                        }
                        int length = socketClient.Receive(data);
                        string message = Encoding.UTF8.GetString(data, 0, length);
                        // Debug.Log("Recieve message: " + message);

                        char[] seps = {'\r', '\n'};
                        string[] quatStrings = message.Split(seps);
                        if (quatStrings.Length < 1) continue;
                        string[] quaternion = quatStrings[0].Split(' ');
                        if (quaternion.Length != 5) continue;
                        isTrigger = quaternion[0].CompareTo("1") == 0;
                        float _w = Convert.ToSingle(quaternion[1]);
                        float _x = Convert.ToSingle(quaternion[2]);
                        float _y = Convert.ToSingle(quaternion[3]);
                        float _z = Convert.ToSingle(quaternion[4]);
                        // w = _w;
                        // x = _x;
                        // y = _y;
                        // z = _z;
                        if (!inited) {
                            x_init = _y;
                            y_init = -_z;
                            z_init = -_x;
                            inited = true;
                        }
                        w = _w;
                        x = _y - x_init;
                        y = -_z - y_init;
                        z = -_x - z_init;
                    }
                } catch (Exception ex) {
                    if (socketClient != null) {
                        socketClient.Close();
                    }
                    Debug.Log(ex.Message);
                }
            }
        });
        thread.IsBackground = true;
        thread.Start();
    }

    public void Close() {
        thread.Abort();
        if (socketClient != null) {
            socketClient.Close();
        }
    }
}
