import pyaudio
import wave
from datetime import datetime


def record_audio(filename, duration=5, sample_rate=44100, channels=2, chunk_size=1024):
    audio_format = pyaudio.paInt16

    audio = pyaudio.PyAudio()

    stream = audio.open(
        format=audio_format,
        channels=channels,
        rate=sample_rate,
        input=True,
        frames_per_buffer=chunk_size,
    )

    print("Recording started...")
    frames = []

    for _ in range(0, int(sample_rate / chunk_size * duration)):
        data = stream.read(chunk_size)
        frames.append(data)

    print("Recording finished.")

    stream.stop_stream()
    stream.close()
    audio.terminate()

    wave_file = wave.open(filename, "wb")
    wave_file.setnchannels(channels)
    wave_file.setsampwidth(audio.get_sample_size(audio_format))
    wave_file.setframerate(sample_rate)
    wave_file.writeframes(b"".join(frames))
    wave_file.close()

    print(f"Audio saved as {filename}")


# Usage example
def main(args=None):
    file_name = "output-" + str(datetime.now()) + ".wav"
    record_audio(file_name, duration=10)


if __name__ == "__main__":
    main()
