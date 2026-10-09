/**
 * Renders the recorded Tk scene (webapp/drawing.py) to a canvas, mapping Tk item semantics:
 * empty fill/outline means no paint, missing option takes the Tk default, dash arrays are in px,
 * stipple gray25 is approximated with fill alpha 0.25, and Tk point font sizes become CSS px at 1.333.
 */

export interface SceneOpts {
  fill?: string
  outline?: string
  width?: number
  dash?: number[] | string | number
  tags?: string[]
  stipple?: string
  text?: string
  anchor?: string
  font?: (string | number)[] | string
  justify?: string
}

export interface SceneItem {
  kind: 'line' | 'polygon' | 'rectangle' | 'oval' | 'text'
  coords: number[]
  opts: SceneOpts
}

/** A draw handle's geometry to stroke over the scene as the hover/drag highlight. */
export interface DrawHighlight {
  kind: 'polygon' | 'line'
  coords: number[]
}

// Tk font sizes are points at ~96 dpi; the browser draws in CSS px.
const PX_PER_TK_POINT = 1.333
const LINE_HEIGHT_FACTOR = 1.2
// Tk stipple patterns are bitmaps; a flat alpha is the documented approximation.
const STIPPLE_ALPHA: Record<string, number> = { gray12: 0.12, gray25: 0.25, gray50: 0.5, gray75: 0.75 }
const DEFAULT_FAMILY = '"DejaVu Sans", "Liberation Sans", sans-serif'
const MONO_FAMILY = '"DejaVu Sans Mono", "Liberation Mono", monospace'

interface ParsedFont {
  css: string
  lineHeight: number
}

function parseFont(font: SceneOpts['font']): ParsedFont {
  let family = ''
  let size = 9
  const styles: string[] = []
  if (typeof font === 'string') {
    family = font
  } else if (Array.isArray(font)) {
    if (typeof font[0] === 'string') family = font[0]
    if (typeof font[1] === 'number') size = font[1]
    for (const part of font.slice(2)) {
      if (part === 'bold' || part === 'italic') styles.push(part)
    }
  }
  // A negative Tk size is already in pixels.
  const px = size < 0 ? -size : size * PX_PER_TK_POINT
  const familyCss = family === 'TkFixedFont' ? MONO_FAMILY : family === '' || family === 'TkDefaultFont' ? DEFAULT_FAMILY : `'${family}', ${DEFAULT_FAMILY}`
  return { css: [...styles, `${px}px`, familyCss].join(' '), lineHeight: px * LINE_HEIGHT_FACTOR }
}

function dashArray(dash: SceneOpts['dash']): number[] {
  if (dash === undefined || dash === '') return []
  const values = typeof dash === 'number' ? [dash] : typeof dash === 'string' ? dash.split(/[ ,]+/).map(Number) : dash.map(Number)
  return values.filter((value) => Number.isFinite(value) && value > 0)
}

export class CanvasRenderer {
  private readonly context: CanvasRenderingContext2D
  private width = 0
  private height = 0
  private dpr = 1
  private lastScene: SceneItem[] | null = null
  private lastHighlight: DrawHighlight | null = null
  private dprQuery: MediaQueryList | null = null

  constructor(private readonly canvas: HTMLCanvasElement) {
    const context = canvas.getContext('2d')
    if (context === null) throw new Error('2D canvas context unavailable')
    this.context = context
    this.watchDevicePixelRatio()
  }

  resize(width: number, height: number): void {
    this.dpr = window.devicePixelRatio || 1
    this.width = width
    this.height = height
    this.canvas.width = Math.max(1, Math.round(width * this.dpr))
    this.canvas.height = Math.max(1, Math.round(height * this.dpr))
  }

  /** Re-check the ratio and repaint the last scene at the new backing size (the browser moved the window to another display). */
  private watchDevicePixelRatio(): void {
    if (this.dprQuery !== null) this.dprQuery.removeEventListener('change', this.onDprChange)
    const query = window.matchMedia(`(resolution: ${window.devicePixelRatio || 1}dppx)`)
    query.addEventListener('change', this.onDprChange)
    this.dprQuery = query
  }

  private readonly onDprChange = (): void => {
    this.watchDevicePixelRatio() // the query string embeds the old ratio; re-match against the new one
    if (this.width > 0 && this.height > 0 && this.lastScene !== null) {
      this.resize(this.width, this.height)
      this.render(this.lastScene)
    }
  }

  dispose(): void {
    if (this.dprQuery !== null) {
      this.dprQuery.removeEventListener('change', this.onDprChange)
      this.dprQuery = null
    }
  }

  /** Paint a scene, keeping it for later highlight-only repaints. A missing highlight keeps the current one. */
  render(items: SceneItem[], highlight: DrawHighlight | null = this.lastHighlight): void {
    this.lastScene = items
    this.lastHighlight = highlight
    this.paintScene()
  }

  /** Restroke only the highlight over the retained scene (hover changes must not rebuild the scene). */
  setHighlight(highlight: DrawHighlight | null): void {
    this.lastHighlight = highlight
    this.paintScene()
  }

  private paintScene(): void {
    const items = this.lastScene
    if (items === null) return
    const ctx = this.context
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0)
    ctx.clearRect(0, 0, this.width, this.height)
    for (const item of items) {
      ctx.globalAlpha = 1
      ctx.lineWidth = 1
      ctx.setLineDash([])
      if (item.kind === 'line') this.drawLine(item)
      else if (item.kind === 'polygon') this.drawPolygon(item)
      else if (item.kind === 'rectangle') this.drawRectangle(item)
      else if (item.kind === 'oval') this.drawOval(item)
      else this.drawText(item)
    }
    if (this.lastHighlight !== null) this.strokeHighlight(this.lastHighlight)
  }

  /** The hover/drag highlight: a solid white outline, no fill change. */
  private strokeHighlight(highlight: DrawHighlight): void {
    const coords = highlight.coords
    if (coords.length < 4) return
    const ctx = this.context
    ctx.beginPath()
    ctx.moveTo(coords[0], coords[1])
    for (let index = 2; index + 1 < coords.length; index += 2) ctx.lineTo(coords[index], coords[index + 1])
    if (highlight.kind === 'polygon') ctx.closePath()
    ctx.globalAlpha = 1
    ctx.setLineDash([])
    ctx.strokeStyle = '#ffffff'
    ctx.lineWidth = 2.5
    ctx.stroke()
  }

  private drawLine(item: SceneItem): void {
    const coords = item.coords
    if (coords.length < 4) return
    const fill = item.opts.fill ?? '#000000' // Tk line default fill
    if (fill === '') return
    const ctx = this.context
    ctx.beginPath()
    ctx.moveTo(coords[0], coords[1])
    for (let index = 2; index + 1 < coords.length; index += 2) ctx.lineTo(coords[index], coords[index + 1])
    ctx.strokeStyle = fill
    ctx.lineWidth = item.opts.width ?? 1
    ctx.setLineDash(dashArray(item.opts.dash))
    ctx.stroke()
  }

  private drawPolygon(item: SceneItem): void {
    const coords = item.coords
    if (coords.length < 6) return
    const ctx = this.context
    ctx.beginPath()
    ctx.moveTo(coords[0], coords[1])
    for (let index = 2; index + 1 < coords.length; index += 2) ctx.lineTo(coords[index], coords[index + 1])
    ctx.closePath()
    this.paint(item.opts)
  }

  private drawRectangle(item: SceneItem): void {
    const coords = item.coords
    if (coords.length < 4) return
    const ctx = this.context
    const x = Math.min(coords[0], coords[2])
    const y = Math.min(coords[1], coords[3])
    ctx.beginPath()
    ctx.rect(x, y, Math.abs(coords[2] - coords[0]), Math.abs(coords[3] - coords[1]))
    this.paint(item.opts)
  }

  private drawOval(item: SceneItem): void {
    const coords = item.coords
    if (coords.length < 4) return
    const radiusX = Math.abs(coords[2] - coords[0]) / 2
    const radiusY = Math.abs(coords[3] - coords[1]) / 2
    if (radiusX <= 0 || radiusY <= 0) return
    const ctx = this.context
    ctx.beginPath()
    ctx.ellipse((coords[0] + coords[2]) / 2, (coords[1] + coords[3]) / 2, radiusX, radiusY, 0, 0, Math.PI * 2)
    this.paint(item.opts)
  }

  private drawText(item: SceneItem): void {
    const coords = item.coords
    const text = item.opts.text
    if (coords.length < 2 || text === undefined) return
    const fill = item.opts.fill ?? '#000000' // Tk text default fill
    if (fill === '') return
    const anchor = item.opts.anchor ?? 'center'
    const font = parseFont(item.opts.font)
    const lines = text.split('\n')
    const ctx = this.context
    ctx.font = font.css
    const boxWidth = Math.max(0, ...lines.map((line) => ctx.measureText(line).width))
    const boxHeight = lines.length * font.lineHeight
    const boxLeft = anchor.endsWith('w') ? coords[0] : anchor.endsWith('e') ? coords[0] - boxWidth : coords[0] - boxWidth / 2
    const top = anchor.startsWith('n') ? coords[1] : anchor.startsWith('s') ? coords[1] - boxHeight : coords[1] - boxHeight / 2
    // Tk justify aligns the lines inside the text's bounding box; default is left.
    const justify = item.opts.justify === 'center' ? 'center' : item.opts.justify === 'right' ? 'right' : 'left'
    ctx.textAlign = justify
    ctx.textBaseline = 'top'
    ctx.fillStyle = fill
    const x = justify === 'center' ? boxLeft + boxWidth / 2 : justify === 'right' ? boxLeft + boxWidth : boxLeft
    for (let index = 0; index < lines.length; index += 1) {
      ctx.fillText(lines[index], x, top + index * font.lineHeight)
    }
  }

  private paint(opts: SceneOpts): void {
    const ctx = this.context
    const fill = opts.fill ?? '' // polygon/rectangle/oval default fill is none
    if (fill !== '') {
      ctx.fillStyle = fill
      ctx.globalAlpha = STIPPLE_ALPHA[opts.stipple ?? ''] ?? 1
      ctx.fill()
      ctx.globalAlpha = 1
    }
    const outline = opts.outline ?? '#000000' // Tk outline default is black
    if (outline !== '') {
      ctx.strokeStyle = outline
      ctx.lineWidth = opts.width ?? 1
      ctx.setLineDash(dashArray(opts.dash))
      ctx.stroke()
    }
  }
}
