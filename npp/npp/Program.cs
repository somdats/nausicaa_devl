using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

using System.Drawing;
using System.Drawing.Imaging;
using System.IO;




namespace npp
{
    class Program
    {
        [DllImport("NAUSICAA_API.dll")]
        static public extern void connectToVRServer(StringBuilder sb);


        [DllImport("NAUSICAA_API.dll")]
        static public extern void startStreaming();

        [DllImport("NAUSICAA_API.dll")]
        static public extern int addVirtualCamera();

        [DllImport("NAUSICAA_API.dll")]
        static public extern void renderFromCamera(int param);

        [DllImport("NAUSICAA_API.dll")]
        static public extern IntPtr readFrame(out int bytes_count);

        [DllImport("NAUSICAA_API.dll")]
        static public extern IntPtr getVirtualCameraList(out int bytes_count);

        [DllImport("NAUSICAA_API.dll")]
        static public extern void setCameraFrustrum(int param, float a, float b, float c, float d, float e, int w, int h);

        [DllImport("NAUSICAA_API.dll")]
        static public extern void setPosition(int id, float x, float y, float z);

        [DllImport("NAUSICAA_API.dll")]
        static public extern void setViewDirection(int id, float dx, float dy, float dz);
        static void Main()
        {
            StringBuilder sb = new StringBuilder("146.48.84.241");

            connectToVRServer(sb);

            int camID = addVirtualCamera();
            setCameraFrustrum(camID, -0.2f, 0.2f, -0.2f, 0.2f, 0.2f, 1280, 720);
            setPosition(camID, 2.0f, -1.0f, 2.0f);
            setViewDirection(camID, 0.0f, 0.0f, -1.0f);

            renderFromCamera(camID);
            startStreaming();
            //int size = 3000000 - 4;
            IntPtr raw_bytes = readFrame(out int bytes_count);

            byte[] _imageTemp = new byte[bytes_count];
            Marshal.Copy(raw_bytes, _imageTemp, 0, bytes_count);

            IntPtr cambytes = getVirtualCameraList(out int num_count);

            byte[] _camTemp = new byte[num_count];
            Marshal.Copy(cambytes, _camTemp, 0, num_count);
            for (int i = 0; i < num_count; i++)
            {
                Console.WriteLine(_camTemp[i]);
            }

            //byte[] bytes = Encoding.ASCII.GetBytes(raw_bytes);

            ////MemoryStream stream = new MemoryStream();

            ////stream.Write(bytes, 0, bytes.Length);


            MemoryStream stream = new MemoryStream(_imageTemp);
            //FileStream file = new FileStream("D:/Deliverables/file.dat", FileMode.Create, FileAccess.Write);
            //stream.WriteTo(file);
            //file.Close();
            //ms.Close();
            Image image = Image.FromStream(stream);

            image.Save("D:/Deliverables/test_new.jpg", ImageFormat.Jpeg);  // Or Png


        }

    }
}
