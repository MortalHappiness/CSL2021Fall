using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EnemyController : MonoBehaviour
{
    public Transform playerController;

    public Transform spawnPoints;
    public float spawnTime = 5f;
    public float spawnTimer;

    public GameObject enemyPref;
    public float speed = 0.2f;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (spawnTimer <= 0f)
        {
            spawnTimer = spawnTime;
            int spawnPointIndex = Random.Range(0, spawnPoints.childCount);
            Debug.Log("Spawn at " + spawnPointIndex);
            GameObject obj = GameObject.Instantiate(enemyPref, spawnPoints.GetChild(spawnPointIndex));
            obj.GetComponent<Enemy>().speed = speed;
            obj.GetComponent<Enemy>().target = playerController;
        } else
        {
            spawnTimer = spawnTimer - Time.deltaTime;
        }
    }
}
