import openai
import os
import rclpy
import wave
import numpy as np
import sounddevice as sd
import webrtcvad
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

# Parámetros
SAMPLE_RATE = 16000
FRAME_DURATION = 30  # Duración del frame en ms (10, 20, 30)
CHANNELS = 1
VAD_SENSITIVITY = 0  # Sensibilidad del VAD (0 = más permisivo, 3 = más estricto)
SILENCE_DURATION = 1.5  # Tiempo en segundos de silencio antes de parar

class GSTTService(Node):
    def __init__(self):
        super().__init__("gstt_service_node")

        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            self.get_logger().error("❌ No se encontró la clave de API de OpenAI. Exporta OPENAI_API_KEY antes de ejecutar.")
            exit(1)

        self.srv = self.create_service(SetBool, "gstt_service", self.gstt_callback)
        self.get_logger().info('✅ GSTTService con Whisper API inicializado')

        # Inicializar el detector de actividad de voz (VAD)
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(VAD_SENSITIVITY)

    def record_audio_with_vad(self):
        self.get_logger().info('🎙 Esperando detección de voz...')

        audio_buffer = []
        last_voice_time = None  # No inicia temporizador hasta que se detecta voz

        stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, dtype=np.int16)
        with stream:
            while True:
                frame, _ = stream.read(int(SAMPLE_RATE * FRAME_DURATION / 1000))
                frame_bytes = frame.tobytes()  # Convertir a bytes

                is_speech = self.vad.is_speech(frame_bytes, SAMPLE_RATE)

                if is_speech:
                    if last_voice_time is None:  # Primera detección de voz
                        self.get_logger().info('🔊 ¡Detección de voz iniciada! Comenzando grabación...')
                    last_voice_time = time.time()
                    audio_buffer.append(frame)
                elif last_voice_time is not None and time.time() - last_voice_time > SILENCE_DURATION:
                    self.get_logger().info('🛑 Se detectó silencio prolongado. Terminando grabación.')
                    break

        if audio_buffer:
            audio_data = np.concatenate(audio_buffer, axis=0)
            return audio_data
        else:
            return np.array([])

    def gstt_callback(self, sRequest, sResponse):
        if sRequest.data:
            try:
                audio_data = self.record_audio_with_vad()

                if len(audio_data) == 0:
                    self.get_logger().info("⚠️ No se detectó voz.")
                    sResponse.success = True
                    sResponse.message = "No se detectó voz en la grabación."
                    return sResponse

                # Guardar en un archivo WAV
                audio_path = "/tmp/audio.wav"
                with wave.open(audio_path, "wb") as wf:
                    wf.setnchannels(CHANNELS)
                    wf.setsampwidth(2)
                    wf.setframerate(SAMPLE_RATE)
                    wf.writeframes(audio_data.tobytes())

                self.get_logger().info("🔍 Enviando audio a OpenAI Whisper API")

                # Enviar a Whisper para transcripción
                with open(audio_path, "rb") as audio_file:
                    client = openai.OpenAI(api_key=self.api_key)
                    response = client.audio.transcriptions.create(
                        model="whisper-1",
                        file=audio_file,
                        language="es",
                    )

                sResponse.success = True
                sResponse.message = response.text
                self.get_logger().info(f'📝 Transcripción: {sResponse.message}')

            except Exception as e:
                self.get_logger().error(f'❌ Error en el reconocimiento de voz: {e}')
                sResponse.success = False
                sResponse.message = str(e)

            return sResponse
        else:
            sResponse.success = False
            sResponse.message = "El servicio debe llamarse con 'True' para iniciar reconocimiento."
            return sResponse

def main():
    rclpy.init()
    gstt_service = GSTTService()
    rclpy.spin(gstt_service)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
