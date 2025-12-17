<!-- --- title: "Voice-to-Action: Using OpenAI Whisper for Voice Commands"
sidebar_position: 2
tags: [Whisper, OpenAI, Speech-to-Text, STT, ROS 2, HRI]
---

# Chapter 12: Voice-to-Action: Using OpenAI Whisper for Voice Commands

The most intuitive way to communicate with a humanoid robot is to simply talk to it. This chapter focuses on the first critical step in our Vision-Language-Action pipeline: converting human speech into machine-readable text. We will leverage **OpenAI's Whisper**, a state-of-the-art, open-source Speech-to-Text (STT) model, to build a robust voice command interface for our robot.

### Comprehensive Introduction: The Importance of High-Fidelity STT

For a robot to understand commands, it must first *hear* them accurately. Poor transcription quality is a critical failure point in any voice-controlled system. A command like "Bring me the *pear*" being misheard as "Bring me the *bear*" could have disastrous consequences. Whisper's high accuracy, robustness to background noise, and ability to handle various accents make it an ideal choice for robotics applications where clarity and reliability are paramount. In this chapter, we will walk through setting up Whisper, capturing real-time audio, and integrating it into our ROS 2 ecosystem to create a "speech listener" node.

---

## How Whisper Works: A High-Level View

Whisper is a deep learning model trained on a massive dataset of diverse audio. It processes audio by first converting it into a **log-Mel spectrogram**—a visual representation of the spectrum of frequencies as they vary over time. This spectrogram is then fed into a sophisticated **Transformer neural network**, which analyzes the patterns and outputs the corresponding text transcription.

<!-- ![Figure 12-1: The Whisper Transcription Pipeline](img/whisper_pipeline.png) -->
*This diagram shows the flow of data in the Whisper model. Raw audio is captured and converted into a log-Mel spectrogram. This image-like representation is then processed by a Transformer encoder-decoder network, which ultimately outputs the transcribed text.*

> **Best Practice: Model Selection**
> Whisper comes in various sizes. For robotics, choosing the right model is a trade-off between speed and accuracy. Smaller models are faster but less accurate, while larger models are more accurate but computationally expensive. A good starting point for real-time applications on a GPU-equipped robot is the `base` or `small` model.

### Table 1: Whisper Model Comparison

| Model Size | Parameters | Relative Speed | VRAM Usage (Approx.) | Use Case                                       |
| :--------- | :--------- | :------------- | :------------------- | :--------------------------------------------- |
| `tiny`     | ~39 M      | ~32x           | ~1 GB                | Low-resource devices, offline transcription.   |
| `base`     | ~74 M      | ~16x           | ~1 GB                | Good balance for real-time tasks on decent hardware. |
| `small`    | ~244 M     | ~6x            | ~2 GB                | Excellent choice for GPU-accelerated real-time HRI. |
| `medium`   | ~769 M     | ~2x            | ~5 GB                | High-accuracy transcription, near human-level. |
| `large`    | ~1550 M    | 1x             | ~10 GB               | Maximum accuracy, typically for offline analysis. |

---

## Step 1: Basic Transcription from a File

Before diving into real-time audio, let's start with the simplest use case: transcribing a pre-recorded audio file. This helps verify our installation and understand the core API.

### Setup

First, install the necessary libraries.

```bash
# Install OpenAI's Whisper and a library for audio manipulation
pip install openai-whisper pydub
```

### Code Example 1: Transcribing a WAV File

```python
# file: transcribe_file.py
import whisper
from pydub import AudioSegment
import os

def transcribe_audio_file(file_path):
    """
    Transcribes a given audio file using the Whisper 'base' model.
    
    Args:
        file_path (str): The path to the audio file (e.g., .wav, .mp3).
    """
    # --- 1. Load the Whisper Model ---
    # This line loads the specified model into memory. The first time you run this,
    # it will download the model weights. Subsequent runs will be faster.
    # We choose 'base' for a good balance of speed and accuracy.
    model = whisper.load_model("base")
    print("Whisper 'base' model loaded successfully.")

    # --- 2. Load and Prepare the Audio ---
    # Whisper works best with 16kHz mono audio. We use pydub to ensure our audio
    # file matches this format, converting it if necessary.
    print(f"Loading audio file: {file_path}")
    audio = AudioSegment.from_file(file_path)
    audio = audio.set_frame_rate(16000).set_channels(1)
    
    # Create a temporary file for the converted audio
    temp_file_path = "temp_16khz_mono.wav"
    audio.export(temp_file_path, format="wav")
    print(f"Audio converted to 16kHz mono and saved to {temp_file_path}")

    # --- 3. Transcribe the Audio ---
    # The model.transcribe() function takes the audio file path and performs the
    # transcription. It returns a dictionary containing the text and other metadata.
    print("Transcribing audio... this may take a moment.")
    result = model.transcribe(temp_file_path)
    
    # --- 4. Print the Result and Clean Up ---
    # We access the transcribed text via the 'text' key in the result dictionary.
    transcribed_text = result["text"]
    print("\n--- Transcription Result ---")
    print(transcribed_text)
    print("--------------------------\n")
    
    # Clean up the temporary file
    os.remove(temp_file_path)
    print(f"Cleaned up temporary file: {temp_file_path}")

if __name__ == '__main__':
    # Create a dummy audio file for testing if you don't have one.
    # For a real test, record yourself saying a command and save it as "command.wav".
    if not os.path.exists("command.wav"):
        print("Creating a dummy 'command.wav'. Please replace it with a real audio file.")
        dummy_audio = AudioSegment.silent(duration=2000) # 2 seconds of silence
        dummy_audio.export("command.wav", format="wav")
        
    transcribe_audio_file("command.wav")
```
This script demonstrates the fundamental workflow: load the model, prepare the audio, and call the `transcribe` function.

---

## Step 2: A ROS 2 Node for Real-Time Speech Recognition

Now, let's build the core component: a ROS 2 node that listens to a microphone in real-time and publishes transcriptions.

### Setup

We need a library to access the microphone. `sounddevice` is an excellent choice.

```bash
# Install libraries for audio capture and numerical operations
pip install sounddevice numpy
```

### Code Example 2: The `speech_to_text_publisher` Node

```python
# file: speech_to_text_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import numpy as np
import sounddevice as sd
import queue
import threading

class SpeechToTextPublisher(Node):
    def __init__(self):
        super().__init__('speech_to_text_publisher')
        
        # --- 1. ROS 2 Publisher ---
        # Create a publisher to broadcast the transcribed text on the '/voice_command' topic.
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        self.get_logger().info("ROS 2 Publisher on '/voice_command' is ready.")

        # --- 2. Whisper Model Loading ---
        # Load the desired Whisper model. Using 'base.en' ensures it's optimized for English.
        self.get_logger().info("Loading Whisper 'base.en' model...")
        self.model = whisper.load_model("base.en")
        self.get_logger().info("Whisper model loaded.")

        # --- 3. Audio Stream Configuration ---
        # Define audio parameters. 16000Hz is required by Whisper.
        self.samplerate = 16000
        self.channels = 1
        # A queue to hold audio data between the audio callback and the processing thread.
        self.audio_queue = queue.Queue()

        # --- 4. Start the Audio Stream and Processing Thread ---
        # A separate thread will handle the CPU-intensive transcription process,
        # so the audio callback remains non-blocking and responsive.
        self.processing_thread = threading.Thread(target=self.process_audio_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.stream = sd.InputStream(
            callback=self.audio_callback,
            samplerate=self.samplerate,
            channels=self.channels,
            dtype='int16' # 16-bit integers are a standard audio format.
        )
        self.stream.start()
        self.get_logger().info("Microphone stream started. Listening for voice commands...")

    def audio_callback(self, indata, frames, time, status):
        """
        This function is called by the sounddevice library for each new chunk of audio data.
        """
        # Simply put the new audio data into the queue to be processed by the other thread.
        self.audio_queue.put(indata.copy())

    def process_audio_queue(self):
        """
        This function runs in a separate thread, continuously processing audio from the queue.
        """
        while rclpy.ok():
            try:
                # --- 5. Accumulate Audio Data ---
                # We'll accumulate a few seconds of audio before transcribing to provide context.
                # This helps avoid transcribing short noises or single words.
                audio_data = []
                # Wait for the first piece of audio data to start accumulating.
                audio_data.append(self.audio_queue.get(timeout=1.0)) 
                
                # Accumulate for ~3 seconds. Adjust duration as needed.
                while len(audio_data) < (self.samplerate / 1024 * 3): # 1024 is a common buffer size
                    audio_data.append(self.audio_queue.get(timeout=0.5))

                # Concatenate the chunks into a single NumPy array.
                audio_np = np.concatenate(audio_data)
                
                # Convert the int16 data to a float32 array between -1 and 1, which Whisper expects.
                audio_float32 = audio_np.flatten().astype(np.float32) / 32768.0

                # --- 6. Transcribe and Publish ---
                self.get_logger().info("Processing accumulated audio...")
                result = self.model.transcribe(audio_float32)
                transcribed_text = result['text'].strip()

                # Only publish if the transcription is not empty or just whitespace.
                if transcribed_text:
                    self.get_logger().info(f'Transcription: "{transcribed_text}"')
                    msg = String()
                    msg.data = transcribed_text
                    self.publisher_.publish(msg)
                else:
                    self.get_logger().info("Transcription was empty, not publishing.")
            
            except queue.Empty:
                # This is expected when there's silence. We just continue waiting.
                continue
            except Exception as e:
                self.get_logger().error(f"An error occurred in the processing thread: {e}")


def main(args=None):
    rclpy.init(args=args)
    stt_publisher = SpeechToTextPublisher()
    rclpy.spin(stt_publisher)
    stt_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

> **Warning: CPU Usage**
> Running this real-time transcription loop, even with the `base` model, can be CPU-intensive. If you notice performance issues, consider adding a "wake word" detection step before transcribing, so that Whisper only runs when the robot is explicitly addressed.

---

## Step 3: Subscribing to the Voice Command

Finally, to show the end-to-end connection, let's create a simple subscriber node that listens for the published commands.

### Code Example 3: The `command_subscriber` Node

```python
# file: command_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')
        
        # --- 1. ROS 2 Subscriber ---
        # Create a subscriber that listens to the '/voice_command' topic.
        # The callback function `listener_callback` will be executed for each message.
        self.subscription = self.create_subscription(
            String,
            'voice_command',
            self.listener_callback,
            10)
        self.get_logger().info("ROS 2 Subscriber on '/voice_command' is ready.")

    def listener_callback(self, msg):
        """
        This function is executed every time a message is received on the topic.
        """
        # Log the received data to the console.
        # In a real application, this is where you would trigger the LLM planner.
        self.get_logger().info(f'I heard the command: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    command_subscriber = CommandSubscriber()
    # rclpy.spin() keeps the node alive to receive messages.
    rclpy.spin(command_subscriber)
    command_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
To run the full system, you would execute the publisher node in one terminal and the subscriber node in another. As you speak, you will see the transcribed text appear in the subscriber's terminal, confirming that your voice commands are successfully being broadcast across the ROS 2 graph.

---

### Summary

In this chapter, we built the first crucial component of our VLA pipeline: a voice interface. We explored the power and architecture of OpenAI's Whisper model and implemented it in a real-world robotics context. Starting with a simple file-based transcription, we progressed to a complete, real-time ROS 2 node that captures microphone audio, transcribes it with high accuracy, and publishes the resulting text command for other parts of the robot's brain to consume. This "speech listener" node serves as the ears of our humanoid, enabling it to receive natural language commands and setting the stage for cognitive planning, which we will explore in the next chapter. -->
