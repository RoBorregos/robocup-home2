from openai import OpenAI
import os

client = OpenAI(
    base_url = 'http://localhost:11434/v1',
    api_key='ollama', # required, but unused
)

# Function to read the file
def read_file(file_path):
    with open(file_path, 'r') as file:
        return file.read()

# Function to summarize the text using OpenAI API
def summarize_code(file_content):

    prompt = f"Provide a detailed summary focusing on code documentation from the following text:\n\n{file_content}"
    
    response = client.chat.completions.create(
        model="llama3.2",  # You can use other models like gpt-3.5-turbo
        messages=[
            {"role": "system", "content": prompt},
            {"role": "user", "content": file_content},
        ],
    )

    return response.choices[0].message.content.strip()

# Function to iterate through files in a directory
def process_directory(dir_path, extensions, output_file):
    with open(output_file, 'w') as out_file:
        for root, dirs, files in os.walk(dir_path):
            for file in files:
                if file.endswith(tuple(extensions)):
                    file_path = os.path.join(root, file)
                    print(f"Processing file: {file_path}")
                    file_content = read_file(file_path)
                    summary = summarize_code(file_content)
                    out_file.write(f"Summary for {file_path}:\n\n")
                    out_file.write(summary)
                    out_file.write("\n\n" + "="*80 + "\n\n")
    print(f"Summaries saved to {output_file}")

# Example usage
if __name__ == "__main__":
    input_dir = "vision/ws/src"  # Replace with your directory path
    output_file = "summaries_output.txt"  # File to save the summaries
    extensions = ['.py', '.cpp']  # List of file extensions to process
    process_directory(input_dir, extensions, output_file)
