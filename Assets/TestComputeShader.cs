using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

public class TestComputeShader : MonoBehaviour
{
    public Vector2Int defaultSize = new Vector2Int(256, 256);
    public bool useDynamicSizing = true;
    public ComputeShader shader;
    public int threadsMax;

    private Texture2D texture;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {

    }

    // Called whenever the screen size changes
    void OnGUI()
    {
        if (Event.current.type.Equals(EventType.Repaint))
        {
            int width = useDynamicSizing ? Screen.width : defaultSize.x;
            int height = useDynamicSizing ? Screen.height : defaultSize.y;
            int threadsX = width;
            int threadsY = height;
            if (threadsMax > 0)
            {
                threadsX = (threadsMax < width) ? threadsMax : width;
                threadsY = (threadsMax < height) ? threadsMax : width;
            }
            texture = recomputeShaderImage(threadsX, threadsY, shader);
            Graphics.DrawTexture(new Rect(0, 0, width, height), texture);
        }
    }

    Texture2D recomputeShaderImage(int width, int height, ComputeShader shader)
    {
        //Set up output texture
        Texture2D tex = new Texture2D(width, height);
        tex.filterMode = FilterMode.Point;

        //Set up compute buffer
        byte[] data = new byte[width * height * 4];
        ComputeBuffer buffer = new ComputeBuffer(data.Length, sizeof(byte)*4);
        buffer.SetData(data);

        Color32 colA = new Color32(255, 0, 0, 255);
        Color32 colB = new Color32(0, 255, 0, 255);

        int[] metadata = { width, height, packRGBA(colA.r, colA.g, colA.b, colA.a), packRGBA(colB.r, colB.g, colB.b, colB.a)};
        ComputeBuffer metadataBuffer = new ComputeBuffer(metadata.Length, sizeof(int));
        metadataBuffer.SetData(metadata);

        //Set up shader
        int kernelHandle = shader.FindKernel("CSMain");
        shader.SetBuffer(kernelHandle, "result", buffer);
        shader.SetBuffer(kernelHandle, "params", metadataBuffer);

        //Deploy Shader
        shader.Dispatch(kernelHandle, width, height, 1);

        //Recieve Data
        buffer.GetData(data);
        Color32[] cols = new Color32[width * height];
        Color32 col = (Color32)(Color.black);
        for (int i = 0; i < data.Length; i += 4)
        {
            //Debug.Log(string.Format("{0},{1},{2},{3}", data[i], data[i + 1], data[i + 2], data[i + 3]));
            col = new Color32(data[i], data[i + 1], data[i + 2], data[i + 3]);
            cols[i / 4] = col;
        }
        tex.SetPixels32(cols);
        tex.Apply();
        buffer.Release();
        metadataBuffer.Release();
        return tex;
    }

    int packRGBA(int r, int g, int b, int a)
    {
        return r | (g << 8) | (b << 16) | (a << 24);
    }
    
}

