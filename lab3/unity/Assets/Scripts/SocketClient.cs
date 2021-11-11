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
                        //Debug.Log("Recieve message: " + message);
                        // */
                        char[] seps = {' '};
                        string[] quatString = message.Split(seps);
                        //Debug.Log("Received element # :" + quatString.Length);
                        /*for(int i = 0; i < quatString.Length; i++){
                            Debug.Log(quatString[i]);
                        }*/
                        if(quatString.Length % 5 != 0){
                            Debug.Log("Failed to parse message");
                            continue;
                        }
                        else{
                            int i = 0;
                            while(i < quatString.Length){
                                isTrigger = (Convert.ToDecimal(quatString[i++]) == 1)? true : false;
                                w = Convert.ToSingle(quatString[i++]);
                                x = Convert.ToSingle(quatString[i++]);
                                y = Convert.ToSingle(quatString[i++]);
                                z = Convert.ToSingle(quatString[i++]);

                            }
                        }
                        isTrigger = false;
                        x = y = z = w = 0;
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
