using UnityEditor.Rendering.LookDev;
using UnityEngine;

public class TestImage : MonoBehaviour
{
    // Draws a texture on the screen at 10, 10 with 100 width, 100 height.

    public bool stretchImage = true;
    public Texture2D aTexture;
    Texture2D texture;
    public Color blend1;
    public Color blend2;

    public Vector2Int internalResolution = new Vector2Int();

    void Start()
    {
        texture = new Texture2D(internalResolution.x, internalResolution.y);
        Color[] pixels = new Color[internalResolution.x * internalResolution.y];
        for (int i = 0; i < pixels.Length; i++)
        {
            int x = i % internalResolution.x;
            int y = i / internalResolution.x;

            pixels[i] = Color.Lerp(Color.black, blend1, ((float)x / internalResolution.x)) + Color.Lerp(Color.black, blend2, ((float)y / internalResolution.y));
        }
        texture.SetPixels(pixels);
        texture.Apply();
    }

    void OnGUI()
    {
        if (Event.current.type.Equals(EventType.Repaint))
        {
            Graphics.DrawTexture(new Rect(0, 0, stretchImage ? Screen.width : internalResolution.x, stretchImage ? Screen.height: internalResolution.y), texture);
        }
    }
}