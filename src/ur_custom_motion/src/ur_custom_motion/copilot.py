from transformers import pipeline

class CoPilot:
    def __init__(self):
        self.model = pipeline("text-generation", model="mistralai/Mistral-7B-Instruct-v0.2", token = 'placeholder_token')

    def generate(self, prompt = """ """):
        prompt = """ 
        Function to generate joint motion given two joint states and a velocity and acceleration is available. \n
        An example of the function usage is given below: \n
        example: handle_joint_motion([0.5, -0.3, 1.2, 0.0 , 0.0 , 0.0 ], [0.0, -1.5, 0.0, -1.5, 0.0, 0.0], 0.8, 0.2)\n
        Give more examples of the function usage. \n
       """
        return self.model(prompt, max_length=1000, do_sample=True)[0]["generated_text"]
    
    def suggest_code(self):
        code_suggestion = self.generate()
        return code_suggestion