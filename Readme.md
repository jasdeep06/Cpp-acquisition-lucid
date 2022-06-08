Synchronous acquisition of images from both the cameras at 6 fps each.

1. The main thread acts as acquisition thread.
2. There is one polling thread that polls buffer from both the cameras one after another,creates a copy of them and enqueues those copies in a queue.The original buffer is requeued to the camera to use again.
3. There are 6 saving threads that dequque the buffern from the queue,convert it to BGR8 from  BayerRG8 and saves it as ".jpg" with quality 75.
4. The above configuration is just enough on a Xavier 32 GB which has a 8 core, 2.3GHz carmel CPU (https://www.nvidia.com/en-in/autonomous-machines/embedded-systems/jetson-agx-xavier/).
5. Each image in its jpg form has a size of approximately 1 MB.