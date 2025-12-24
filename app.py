import streamlit as st
from llama_index.core import SimpleDirectoryReader, VectorStoreIndex, Settings
from llama_index.llms.ollama import Ollama
from llama_index.embeddings.huggingface import HuggingFaceEmbedding
from llama_index.core import PromptTemplate

# Load index with caching
@st.cache_resource(show_spinner=False)
def load_index():
    # Load all .md files from the docs folder
    reader = SimpleDirectoryReader(input_dir="docs/", recursive=True, required_exts=[".md"])
    docs = reader.load_data()

    # Embedding model (small, fast, good quality)
    embed_model = HuggingFaceEmbedding(model_name="BAAI/bge-small-en-v1.5")
    Settings.embed_model = embed_model

    # LLM via Ollama
    llm = Ollama(model="llama3.2:1b", request_timeout=120.0 , temperature=0.1) 
    Settings.llm = llm

    # Create vector index
    index = VectorStoreIndex.from_documents(docs)

    # Custom QA prompt to keep responses grounded
    qa_prompt_tmpl_str = (
        "You are a helpful assistant answering questions about the Physical AI book. "
        "Use ONLY the context provided below to answer. "
        "If the answer is not in the context, respond with: 'Not covered in the book.'\n\n"
        "Context:\n"
        "---------------------\n"
        "{context_str}\n"
        "---------------------\n\n"
        "Question: {query_str}\n"
        "Answer: "
    )
    qa_prompt = PromptTemplate(qa_prompt_tmpl_str)

    # Create query engine with streaming and custom prompt
    query_engine = index.as_query_engine(
        streaming=True,
        similarity_top_k=4,
        text_qa_template=qa_prompt  # Correct way to set custom prompt
    )

    return query_engine


# Load the index (cached)
query_engine = load_index()

# Streamlit UI
st.title("🤖 Physical AI & Humanoid Robots Book Assistant")
st.caption("Ask questions — answers are grounded in your course book content!")

# Initialize chat history
if "messages" not in st.session_state:
    st.session_state.messages = []

# Display chat messages
for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        st.markdown(message["content"])

# Chat input
if query := st.chat_input("Ask a question about Physical AI or humanoid robots..."):
    # Add user message
    st.session_state.messages.append({"role": "user", "content": query})
    with st.chat_message("user"):
        st.markdown(query)

    # Generate and stream assistant response
    with st.chat_message("assistant"):
        with st.spinner("Searching the book and thinking..."):
            # Placeholder for streaming output
            response_placeholder = st.empty()
            full_response = ""

            # Stream the response token by token
            streaming_response = query_engine.query(query)
            
            for token in streaming_response.response_gen:
                full_response += token
                response_placeholder.markdown(full_response + "▌")  # Cursor effect

            # Final clean response (without cursor)
            response_placeholder.markdown(full_response)

    # Save assistant response to history
    st.session_state.messages.append({"role": "assistant", "content": full_response})