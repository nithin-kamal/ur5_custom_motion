from transformers import pipeline

class CoPilot:
    def __init__(self):
        self.model = pipeline("text-generation", model="EleutherAI/gpt-neo-2.7B")

    def generate(self, prompt = """ """):
        prompt = """
        I have a custom ROS node with an API that includes: 
        1. Function to generate joint motion given two joint states and a velocity and acceleration
        2. Function to generate pose motion given two poses and a velocity and acceleration
        3. Function to get the robot state"""
        return self.model(prompt, max_length=100, do_sample=True)[0]["generated_text"]
    
    def suggest_code(self):
        code_suggestion = self.generate()
        return code_suggestion