import { createRoot } from 'react-dom/client'
import { App } from './App'
import './theme.css'

const host = document.getElementById('root')
if (host === null) throw new Error('index.html has no #root element')
createRoot(host).render(<App />)
